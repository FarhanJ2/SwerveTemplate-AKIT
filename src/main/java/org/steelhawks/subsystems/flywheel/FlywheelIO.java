package org.steelhawks.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    class FlywheelIOInputs {
        public double bottomVelocityRPM = 0.0;
        public double topVelocityRPM = 0.0;
        public double appliedVolts = 0.0;
    }

    default void updateInputs(FlywheelIOInputs inputs) {}

    /** Used for SYSID Regression */
    default void setVoltage(double topFlywheelVolts, double bottomFlywheelVolts) {}
    default void setVelocity(double velocityRPM, double topFfVolts, double bottomFfVolts) {}
    default void stop() {}
    default void configureBottomFlywheelPID(double kP, double kI, double kD) {}
    default void configureTopFlywheelPID(double kP, double kI, double kD) {}
}
