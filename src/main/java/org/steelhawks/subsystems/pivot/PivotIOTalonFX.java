package org.steelhawks.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLogOutput;
import org.steelhawks.Constants;

public class PivotIOTalonFX implements PivotIO {

    private final TalonFX mPivotMotor;
    private final CANcoder mPivotEncoder;

    private final StatusSignal<Double> pivotMotorAppliedVolts;
    private final StatusSignal<Double> pivotMotorVelocity;
    private final StatusSignal<Double> pivotMotorCurrent;

    public PivotIOTalonFX() {
        mPivotMotor = new TalonFX(KPivot.PIVOT_ID, Constants.CANIVORE_NAME);
        mPivotEncoder = new CANcoder(KPivot.PIVOT_ENCODER_ID, Constants.CANIVORE_NAME);

        pivotMotorAppliedVolts = mPivotMotor.getMotorVoltage();
        pivotMotorVelocity = mPivotMotor.getVelocity();
        pivotMotorCurrent = mPivotMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50, pivotMotorAppliedVolts, pivotMotorVelocity, pivotMotorCurrent);

        mPivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            pivotMotorAppliedVolts, pivotMotorVelocity, pivotMotorCurrent);

        inputs.pivotMotorAppliedVolts = pivotMotorAppliedVolts.getValueAsDouble();
        inputs.pivotMotorVelocity = pivotMotorVelocity.getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotorCurrent.getValueAsDouble();
        inputs.pivotPositionRads = getPivotPosition();
    }

    @AutoLogOutput(key = "Pivot/Pivot Position")
    private double getPivotPosition() {
        double degrees = mPivotEncoder.getAbsolutePosition().getValueAsDouble() * 360 - 24.7;
        return Math.toRadians(degrees) + 3.05 - 0.03;
    }

    @Override
    public void runPivot(double appliedVolts) {
        mPivotMotor.setVoltage(appliedVolts);
    }

    @Override
    public void runPivotManual(double joystickValue) {

        if (Math.abs(joystickValue) < KPivot.DEADBAND) {
            joystickValue = 0;
        }

        double appliedVolts = Math.cos(getPivotPosition()) * KPivot.PIVOT_KG; // gravity compensation

        appliedVolts += joystickValue * KPivot.INCREMENT_VOLTAGE; // scale voltage based on joystick value

        if (joystickValue > 0) {
            appliedVolts += KPivot.PIVOT_KS;
        } else if (joystickValue < 0) {
            appliedVolts -= KPivot.PIVOT_KS;
        }

        runPivot(appliedVolts);
    }

    @Override
    public void stopPivot() {
        mPivotMotor.stopMotor();
    }
}
