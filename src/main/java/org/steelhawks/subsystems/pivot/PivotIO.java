package org.steelhawks.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    class PivotIOInputs {
        public double pivotMotorAppliedVolts = 0;
        public double pivotMotorVelocity = 0;
        public double pivotMotorCurrent = 0;

        public double pivotPositionRads = 0;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default void runPivot(double appliedVolts) {}
    default void runPivotManual(double joystickValue) {}
    default void stopPivot() {}
}
