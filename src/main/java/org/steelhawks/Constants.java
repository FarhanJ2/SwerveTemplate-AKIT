package org.steelhawks;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    /** The name of our robot. */
    public static final String ROBOT_NAME = "";
    /** Enables tuning mode, which allows values to be changed on the fly in AdvantageScope */
    public static boolean TUNING_MODE = false;
    /** Enables whether simulation should play a replay of a real robot log. */
    public static boolean IN_REPLAY_MODE = false;

    /** The CAN loop name on Phoenix Tuner */
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
        public static final int AUTON_PORT_1 = 28;
        public static final int AUTON_PORT_2 = 29;
        public static final int AUTON_PORT_3 = 30;
    }

    public static final class Pose {
        public static final class Red {
            public static final Pose2d ORIGIN = new Pose2d(new Translation2d(16.542, 8.014), new Rotation2d(Math.PI));
        }

        public static final class Blue {
            public static final Pose2d ORIGIN = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        }
    }

    public static final class FieldConstants {
        public static final double FIELD_LENGTH = Units.inchesToMeters(651.223);
        public static final double FIELD_WIDTH = Units.inchesToMeters(323.277);

        public static final Pose2d BLUE_SPEAKER_POSE =
            new Pose2d(new Translation2d(0, 5.671689), new Rotation2d());
    }

    public static final class Deadbands {
        public static final double DRIVE_DEADBAND = 0.1;
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
        public static final double minAreaOfTag = .1;
        public static final double maxVisionPoseError = 0.5;

        // Pipeline IDS
        public static final int limelightShooterTagPipeline = 0;
        public static final int limelightArmTagPipeline = 0;
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
    }
}
