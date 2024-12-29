package org.steelhawks.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.steelhawks.Constants;
import org.steelhawks.lib.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Volts;

public class Swerve extends SubsystemBase {
    private static double SPEED_MULTIPLIER = 1.0;

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final PIDController alignPID;

    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

    public final SwerveDrivePoseEstimator mPoseEstimator =
        new SwerveDrivePoseEstimator(KSwerve.SWERVE_KINEMATICS, rawGyroRotation, lastModulePositions, new Pose2d());

    public Swerve(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {

        this.gyroIO = gyroIO;
        modules[0] = new SwerveModule(flModuleIO, 0);
        modules[1] = new SwerveModule(frModuleIO, 1);
        modules[2] = new SwerveModule(blModuleIO, 2);
        modules[3] = new SwerveModule(brModuleIO, 3);

        // Start threads (no-op for each if no signals have been created)
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            () -> KSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()),
            this::runVelocity,
            new HolonomicPathFollowerConfig(
                KSwerve.MAX_LINEAR_SPEED, KSwerve.DRIVE_BASE_RADIUS, new ReplanningConfig()),
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red,
            this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) -> {
                Logger.recordOutput(
                    "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> {
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });

        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> {
                        for (int i = 0; i < 4; i++) {
                            modules[i].runCharacterization(voltage.in(Volts));
                        }
                    },
                    null,
                    this));

        switch (Constants.CURRENT_MODE) {
            case REPLAY, REAL -> alignPID = new PIDController(0.07, 0, 0); // TEST
            case SIM -> alignPID = new PIDController(0.02, 0, 0);
            default -> alignPID = new PIDController(0, 0, 0);
        }

        alignPID.enableContinuousInput(0, 360);
        alignPID.setTolerance(1);
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        for (var module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }
        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps =
            modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                            - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = KSwerve.SWERVE_KINEMATICS.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            mPoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

            Logger.recordOutput("Swerve/Current Command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = KSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, KSwerve.MAX_LINEAR_SPEED);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = KSwerve.MODULE_TRANSLATIONS[i].getAngle();
        }

        KSwerve.SWERVE_KINEMATICS.resetHeadings(headings);
        stop();
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return mPoseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Returns the current gear. Either high or low. */
    public double getMultiplier() {
        return SPEED_MULTIPLIER;
    }

    /** Returns if the robot has capped its speed to low gear. */
    @AutoLogOutput(key = "Swerve/SlowMode")
    public boolean isSlowMode() {
        return SPEED_MULTIPLIER == KSwerve.SLOW_MODE_MULTIPLIER;
    }

    /** Returns the PID controller used for aligning the robot to a pose. */
    public PIDController getAlignPID() {
        return alignPID;
    }

    /** Returns the calculated angle needed to reach the desired pose. */
    public double calculateTurnAngle(Pose2d target, double robotAngle) {
        double tx = target.getX();
        double ty = target.getY();
        double rx = getPose().getX();
        double ry = getPose().getY();

        double requestedAngle = Math.atan((ty - ry) / (tx - rx)) * (180 / Math.PI);
        double calculatedAngle = (180 - robotAngle + requestedAngle);

        return ((calculatedAngle + 360) % 360);
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        mPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        mPoseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    /** Returns a command to toggle the robot's speed multiplier between high and low gear. */
    public Command toggleMultiplier() {
        return Commands.either(
            Commands.runOnce(() -> SPEED_MULTIPLIER = 1.0),
            Commands.runOnce(() -> SPEED_MULTIPLIER = KSwerve.SLOW_MODE_MULTIPLIER),
            this::isSlowMode);
    }

    /** Returns a command to zero the robot's heading. */
    public Command zeroHeading() {
        return Commands.runOnce(
            () ->
                setPose(
                    new Pose2d(getPose().getTranslation(), new Rotation2d())),
                this)
        .ignoringDisable(true);
    }
}
