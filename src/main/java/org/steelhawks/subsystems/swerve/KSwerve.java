package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import org.steelhawks.lib.COTSTalonFXSwerveConstants;
import org.steelhawks.lib.SwerveModuleConstants;

/**
 * All constants for Swerve
 */
public final class KSwerve {

    public static final int PIGEON_ID = 0; // set up for your robot
    public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = // configure with your own robot drivetrain
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Angle Encoder Invert */
    public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

    /* Robot Drivetrain Measurements */
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(26.75);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(26.75);
    public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference; // configure on your own robot

    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
        new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
        new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };

    /* Swerve Drive Kinematics */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

    /* Swerve Module Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio; // configure on your own robot
    public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio; // configure on your own robot

    /* Motor Inverts */
    public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
    public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

    public static final double SLOW_MODE_MULTIPLIER = .2;

    /* Swerve Drive Current Limiting */
    /* Set this on your own robot */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;
    /* Angle Motor PID Values */
    public static final double angleKP = CHOSEN_MODULE.angleKP;
    public static final double angleKI = CHOSEN_MODULE.angleKI;
    public static final double angleKD = CHOSEN_MODULE.angleKD;

    /* Drive Motor PID Values */
    public static final double DRIVE_KP = 0.12;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    public static final double TURN_KP = 13.0; // this might be 100 check with your robot // was 7.0
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 1.0;

    /* Auto Align Values */
    public static final double AUTO_ALIGN_KP = 0.02;
    public static final double AUTO_ALIGN_KI = 0;
    public static final double AUTO_ALIGN_KD = 0;

    /* Drive Motor Feedforward */
    public static final double DRIVE_KS = 0.078838;
    public static final double DRIVE_KV = 2.5819;
    public static final double DRIVE_KA = 0.23783;

    /* Simulator Values */
    public static final double DRIVE_KP_SIM = 0.1;
    public static final double DRIVE_KI_SIM = 0;
    public static final double DRIVE_KD_SIM = 0;

    public static final double TURN_KP_SIM = 10;
    public static final double TURN_KI_SIM = 0;
    public static final double TURN_KD_SIM = 0;

    public static final double DRIVE_KS_SIM = 0;
    public static final double DRIVE_KV_SIM = 0.13;

    public static final double AUTO_ALIGN_KP_SIM = 0.02;
    public static final double AUTO_ALIGN_KI_SIM = 0;
    public static final double AUTO_ALIGN_KD_SIM = 0;

    /* Swerve Profiling Values */

    /**
     * Hypotenuse of the drive base in meters
     */
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);

    /**
     * Meters per Second
     */
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);

    /**
     * Radians per Second
     */
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;


    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-130.17);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 4;
        public static final int angleMotorID = 5;
        public static final int canCoderID = 6;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-51.34);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 9;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-63.90);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 10;
        public static final int angleMotorID = 11;
        public static final int canCoderID = 12;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-175.96);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
}
