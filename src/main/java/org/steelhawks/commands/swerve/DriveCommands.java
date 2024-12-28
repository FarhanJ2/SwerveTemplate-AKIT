// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.steelhawks.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.Constants.*;


import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.steelhawks.RobotContainer;
import org.steelhawks.lib.AllianceFlip;
import org.steelhawks.subsystems.swerve.KSwerve;
import org.steelhawks.subsystems.swerve.Swerve;

public class DriveCommands {

    private static final Swerve s_Swerve = RobotContainer.s_Swerve;
    private static final double DEADBAND = 0.3;

    private DriveCommands() {}

    public static double continuous180To360(double angle) {
        return (angle + 360) % 360;
    }

    @AutoLogOutput(key = "Swerve/RotationSpeed")
    private static double getRotationSpeedFromPID(Pose2d target) {
        double robotHeading = continuous180To360(s_Swerve.getRotation().getDegrees());
        double requestedAngle =
            s_Swerve.calculateTurnAngle(
                target, s_Swerve.getRotation().getDegrees() + (AllianceFlip.shouldFlip() ? 360 : 180));
        double setpoint = (robotHeading + requestedAngle) % 360;

        return (s_Swerve.isSlowMode() ? 5 : 1)
            * s_Swerve
            .getAlignPID()
            .calculate(continuous180To360(s_Swerve.getRotation().getDegrees()), setpoint);
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
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
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                Rotation2d linearDirection =
                    new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square values
                linearMagnitude = linearMagnitude * linearMagnitude;
                linearMagnitude = Math.pow(linearMagnitude, 2);
                omega = Math.copySign(omega * omega, omega);

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
            }, s_Swerve);
    }


    public static Command rotateToAngle(DoubleSupplier requestedAngle) {

        double robotHeading = continuous180To360(s_Swerve.getRotation().getDegrees());
        double setpoint = (robotHeading + requestedAngle.getAsDouble()) % 360;

        return Commands.run(
            () ->
                s_Swerve.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        0, 0,
                        (s_Swerve.isSlowMode()) ? 5 : 1
                            * s_Swerve.getAlignPID().calculate(continuous180To360(s_Swerve.getRotation().getDegrees()), setpoint),
                        AllianceFlip.shouldFlip() ?
                            s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                            : s_Swerve.getRotation())), s_Swerve)
                                .until(() -> RobotContainer.s_Swerve.getAlignPID().atSetpoint());
    }
}
