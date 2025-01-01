package org.steelhawks.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.steelhawks.Constants;

/** A utility to flip positions from the blue alliance to red alliance */
public class AllianceFlip {

    public static double validate(double xCoordinate) {
        return shouldFlip() ? Constants.FieldConstants.FIELD_LENGTH - xCoordinate : xCoordinate;
    }

    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d validate(Translation2d translation) {
        return shouldFlip() ? new Translation2d(validate(translation.getX()), translation.getY()) : translation;
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d validate(Rotation2d rotation) {
        return shouldFlip() ? new Rotation2d(-rotation.getCos(), rotation.getSin()) : rotation;
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d validate(Pose2d pose) {
        return shouldFlip() ? new Pose2d(validate(pose.getTranslation()), validate(pose.getRotation())) : pose;
    }

    public static Translation3d validate(Translation3d translation3d) {
        return shouldFlip() ? new Translation3d(validate(translation3d.getX()),
            translation3d.getY(), translation3d.getZ()) : translation3d;
    }

    /**
     * Decides if a pose should be flipped based on the current alliance color.
     * @return true if the alliance is red, false otherwise
     */
    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }
}
