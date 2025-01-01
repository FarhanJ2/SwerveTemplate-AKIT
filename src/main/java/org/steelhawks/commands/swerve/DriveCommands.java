package org.steelhawks.commands.swerve;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants.*;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.steelhawks.RobotContainer;
import org.steelhawks.lib.AllianceFlip;
import org.steelhawks.lib.Conversions;
import org.steelhawks.subsystems.swerve.KSwerve;
import org.steelhawks.subsystems.swerve.Swerve;

public class DriveCommands {

    private static final Debouncer mSetpointDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private static final Swerve s_Swerve = RobotContainer.s_Swerve;

    // we use offsets because our front of our robot for HawkRider is the Intake not the SHOOTER
    // so we rotate the robot 180/360 degrees to make the front of the robot the shooter depending on the alliance
    private static final int BLUE_ALLIANCE_OFFSET = 180;
    private static final int RED_ALLIANCE_OFFSET = 360;

    private DriveCommands() {}

    /**
     * Calculates the rotation speed from the PID controller based on the target pose.
     *
     * @param target The target pose to rotate to.
     * @return The calculated rotation speed.
     */
    private static double getRotationSpeedFromPID(Pose2d target) {
        double robotHeading = Conversions.continuous180To360(s_Swerve.getRotation().getDegrees());
        double requestedAngle =
            s_Swerve.calculateTurnAngle(
                target, s_Swerve.getRotation().getDegrees() + (AllianceFlip.shouldFlip() ?
                    RED_ALLIANCE_OFFSET
                        : BLUE_ALLIANCE_OFFSET));
        double setpoint = (robotHeading + requestedAngle) % 360;

        double rotationSpeed = (s_Swerve.isSlowMode() ? 5 : 1)
            * s_Swerve
            .getAlignPID()
            .calculate(Conversions.continuous180To360(s_Swerve.getRotation().getDegrees()), setpoint);

        Logger.recordOutput("Align/RotationSpeed", rotationSpeed);
        Logger.recordOutput("Align/Robot Heading", robotHeading);
        Logger.recordOutput("Align/Requested Angle", requestedAngle);
        Logger.recordOutput("Align/Setpoint", setpoint);
        return rotationSpeed;
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     *
     * @param xSupplier Supplier for the x-axis input.
     * @param ySupplier Supplier for the y-axis input.
     * @param omegaSupplier Supplier for the angular velocity input.
     * @param alignToSpeaker Trigger to align to the speaker.
     * @return The command to execute the joystick drive.
     */
    public static Command joystickDrive(
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier,
        Trigger alignToSpeaker) {
        return Commands.run(
            () -> {
                double linearMagnitude =
                    MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), Deadbands.DRIVE_DEADBAND);
                Rotation2d linearDirection =
                    new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Deadbands.DRIVE_DEADBAND);

                // Square values
                linearMagnitude = Math.pow(linearMagnitude, 2);
                omega = Math.copySign(Math.pow(omega, 2), omega);

                if (alignToSpeaker.getAsBoolean()) {
                    Pose2d speaker = AllianceFlip.validate(FieldConstants.BLUE_SPEAKER_POSE);
                    omega = getRotationSpeedFromPID(speaker);
                }

                Translation2d linearVelocity =
                    new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        (linearVelocity.getX() * KSwerve.MAX_LINEAR_SPEED)
                            * s_Swerve.getMultiplier(),
                        (linearVelocity.getY() * KSwerve.MAX_LINEAR_SPEED)
                            * s_Swerve.getMultiplier(),
                        (omega * KSwerve.MAX_ANGULAR_SPEED) * s_Swerve.getMultiplier(),
                        AllianceFlip.shouldFlip() ?
                            s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                                : s_Swerve.getRotation()));
            }, s_Swerve)
                .withName("Teleop Drive");
    }

    /**
     * Command to rotate to a specific angle.
     *
     * @param target The target angle to rotate to. Make sure this is a blue alliance pose if you want it to be flipped correctly.
     * @return The command to rotate to the target angle.
     */
    public static Command rotateToAngle(Pose2d target) {
        return Commands.run(
            () -> {
                Pose2d validatedTarget = AllianceFlip.validate(target);
                double rotationSpeed = getRotationSpeedFromPID(validatedTarget);

                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0, 0,
                        (rotationSpeed * KSwerve.MAX_ANGULAR_SPEED) * s_Swerve.getMultiplier(),
                        AllianceFlip.shouldFlip() ?
                            s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                                : s_Swerve.getRotation()));
            }, s_Swerve)
                .until(() -> mSetpointDebouncer.calculate(s_Swerve.getAlignPID().atSetpoint()))
                    .beforeStarting(Commands.runOnce(() ->
                        getRotationSpeedFromPID(new Pose2d())))// reset PID setpoint
                            .withTimeout(3)
                                .withName("Rotate to Angle");
    }

    /**
     * Command to drive to a specific position.
     *
     * @param target The target position the robot should drive to. Make sure this is a blue alliance pose if you want it to be flipped correctly.
     * @param emergencyStop The supplier that will stop the command if it returns true.
     * @return The command to drive to the target position.
     */
    public static Command driveToPosition(Pose2d target, BooleanSupplier emergencyStop) {
        return AutoBuilder.pathfindToPose(target, AutonConstants.CONSTRAINTS)
            .onlyWhile(() -> s_Swerve.shouldContinuePathfinding(emergencyStop))
                .beforeStarting(
                    () -> s_Swerve.isPathfinding = true)
                        .finallyDo(() -> s_Swerve.isPathfinding = false)
                            .withName("Drive to Position");
    }

    /**
     * Command to drive to a specific path.
     *
     * @param path The path the robot should drive to and then follow.
     * @param emergencyStop The supplier that will stop the command if it returns true.
     * @return The command to drive to the target path.
     */
    public static Command driveToPath(PathPlannerPath path, BooleanSupplier emergencyStop) {
        return AutoBuilder.pathfindThenFollowPath(path, AutonConstants.CONSTRAINTS)
            .onlyWhile(() -> s_Swerve.shouldContinuePathfinding(emergencyStop))
                .beforeStarting(
                    () -> s_Swerve.isPathfinding = true)
                        .finallyDo(() -> s_Swerve.isPathfinding = false)
                            .withName("Drive to Path");
    }

    /**
     * Command to drive to a specific position. This command will not stop until it reaches the target position.
     *
     * @param target The target position the robot should drive to. Make sure this is a blue alliance pose if you want it to be flipped correctly.
     * @return The command to drive to the target position.
     */
    public static Command driveToPosition(Pose2d target) {
        return driveToPosition(target, () -> false);
    }

    /**
     * Command to drive to a specific path. This command will not stop until it reaches the target path.
     *
     * @param path The path the robot should drive to and then follow.
     * @return The command to drive to the target path.
     */
    public static Command driveToPath(PathPlannerPath path) {
        return driveToPath(path, () -> false);
    }

    /**
     * Command to follow a path.
     *
     * @param path The path the robot should follow.
     * @return The command to follow the path.
     */
    public static Command followPath(PathPlannerPath path) {
        return AutoBuilder.followPath(path)
            .withName("Follow Path");
    }
}
