package org.steelhawks.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
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
import org.steelhawks.lib.OdometryImpl;
import org.steelhawks.Constants.*;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Volts;

public class Swerve extends SubsystemBase {
    private static double SPEED_MULTIPLIER = 1.0;

    public static final Lock odometryLock = new ReentrantLock();
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

    private final SwerveDrivePoseEstimator mPoseEstimator =
        new SwerveDrivePoseEstimator(KSwerve.SWERVE_KINEMATICS, rawGyroRotation, lastModulePositions, new Pose2d());

    public boolean isPathfinding = false;

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
        OdometryImpl.setPoseEstimator(mPoseEstimator);

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::setPose,
            () -> KSwerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()),
            this::runVelocity,
            new HolonomicPathFollowerConfig(
                new PIDConstants(
                    AutonConstants.TRANSLATION_KP,
                    AutonConstants.TRANSLATION_KI,
                    AutonConstants.TRANSLATION_KD
                ),
                new PIDConstants(
                    AutonConstants.ROTATION_KP,
                    AutonConstants.ROTATION_KI,
                    AutonConstants.ROTATION_KD
                ),
                KSwerve.MAX_LINEAR_SPEED, KSwerve.DRIVE_BASE_RADIUS, new ReplanningConfig()),
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red,
            this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
            (activePath) ->
                Logger.recordOutput(
                    "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
            (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

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
            case REPLAY, REAL -> alignPID = new PIDController(KSwerve.AUTO_ALIGN_KP, KSwerve.AUTO_ALIGN_KI, KSwerve.AUTO_ALIGN_KD); // TEST
            case SIM -> alignPID = new PIDController(KSwerve.AUTO_ALIGN_KP_SIM, KSwerve.AUTO_ALIGN_KI_SIM, KSwerve.AUTO_ALIGN_KD_SIM);
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
        Logger.processInputs("Swerve/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled DONT DELETE
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

            // If Pigeon2 is connected use real Gyro data else use kinematics
            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                Twist2d twist = KSwerve.SWERVE_KINEMATICS.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            mPoseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

            Logger.recordOutput("Swerve/Current Command",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "None");

            Logger.recordOutput("Swerve/Is Pathfinding", isPathfinding);
        }
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = KSwerve.SWERVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, KSwerve.MAX_LINEAR_SPEED);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // the module returns the optimized state used for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
        }

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
    @AutoLogOutput(key = "SwerveStates/Velocities")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Positions")
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

    /**
     * Returns true if the robot should continue pathfinding, false if interrupted by driver.
     */
    public boolean shouldContinuePathfinding(BooleanSupplier stopCondition) {
        Logger.recordOutput("Swerve/ShouldInterruptPathfinding", stopCondition.getAsBoolean());
        return !stopCondition.getAsBoolean();
    }

    /** Returns the PID controller used for aligning the robot to a pose. */
    public PIDController getAlignPID() {
        return alignPID;
    }

    /**
     * Calculates the turn angle needed to face a pose, considering the wanted robot heading too.
     *
     * @param target The target pose to rotate to.
     * @param requestedRobotHeading The heading the robot should be facing.
     * @return The calculated turn angle.
     */
    public double calculateTurnAngle(Pose2d target, double requestedRobotHeading) {
        double tx = target.getX();
        double ty = target.getY();
        double rx = getPose().getX();
        double ry = getPose().getY();

        double requestedAngle = Math.atan((ty - ry) / (tx - rx)) * (180 / Math.PI);
        double calculatedAngle = (180 - requestedRobotHeading + requestedAngle);

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
