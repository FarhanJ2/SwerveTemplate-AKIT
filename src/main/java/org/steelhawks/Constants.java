package org.steelhawks;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    /** The name of our robot. */
    public static final String ROBOT_NAME = "";
    /** Enables tuning mode, which allows values to be changed on the fly in AdvantageScope */
    public static boolean TUNING_MODE = false;
    /** Enables whether simulation should play a replay of a real robot log. */
    public static boolean IN_REPLAY_MODE = false;

    /** The CAN bus name on Phoenix Tuner */
    public static String CANIVORE_NAME = "canivore";

    /**
     * This defines the runtime mode used by AdvantageKit. The mode is always "real" when running
     * on a roboRIO. Change the value of "IN_REPLAY_MODE" to switch between "sim" (physics sim) and "replay"
     * (log replay from a file).
     * Right now, it is {@value #IN_REPLAY_MODE} that it will be in replay mode.
     */
    public static Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : IN_REPLAY_MODE ? Mode.REPLAY : Mode.SIM;

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class OIConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    /** Auton Selector DIO Ports */
    public static final class SelectorConstants {
        public static final int PORT_01 = 28;
        public static final int PORT_02 = 29;
        public static final int PORT_03 = 30;
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
        public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);


        public static final Pose2d BLUE_ORIGIN = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        public static final Pose2d RED_ORIGIN = new Pose2d(new Translation2d(16.542, 8.014), new Rotation2d(Math.PI));

        /*
         * To properly use the auto flip feature, the poses MUST be for the blue alliance.
         * The auto flip feature will automatically flip the poses for the red alliance.
         */
        public static final Pose2d BLUE_SPEAKER_POSE =
            new Pose2d(new Translation2d(0, 5.671689), new Rotation2d());
    }

    public static final class Deadbands {
        public static final double DRIVE_DEADBAND = 0.3;
    }

    public static final class LEDConstants {
        public static final int LED_STRIP_LENGTH = 40;
        public static final int LED_PORT = 0;
    }

    /** Limelight configs and names */
    public static class LimelightConstants {
        public static final String limelightShooter = "limelight-shooter";
        public static final String limelightArm = "limelight-arm";

        // Tracking constants
        public static final double MIN_AREA_OF_TAG = .1;
        public static final double MAX_VISION_POSE_ERROR = 0.5;

        // Pipeline IDS
        public static final int LIMELIGHT_SHOOTER_TAG_PIPELINE = 0;
        public static final int LIMELIGHT_ARM_TAG_PIPELINE = 0;
    }

    /** Vision Estimates for Odometry Config */
    public static class PoseConfig {
        // Increase these numbers to trust your model's state estimates less.
        public static final double POSITION_STD_DEV_X = 0.1;
        public static final double POSITION_STD_DEV_Y = 0.1;
        public static final double POSITION_STD_DEV_THETA = 50; // 10

        // Increase these numbers to trust global measurements from vision less.
        public static final double VISION_STD_DEV_X = 5;
        public static final double VISION_STD_DEV_Y = 5;
        public static final double VISION_STD_DEV_THETA = Double.MAX_VALUE;
    }

    public static class AutonConstants {
        public static final double TRANSLATION_KP = 0.0;
        public static final double TRANSLATION_KI = 0.0;
        public static final double TRANSLATION_KD = 0.0;

        public static final double ROTATION_KP = 0.0;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.0;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1.0;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 1.0;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 1.0;

        public static final PathConstraints CONSTRAINTS =
            new PathConstraints(
                MAX_VELOCITY_METERS_PER_SECOND,
                MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    }
}
