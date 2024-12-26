package org.steelhawks.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import org.steelhawks.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {

    private final TalonFX mBottomFlywheel;
    private final TalonFX mTopFlywheel;
    private final TalonFX mFeederMotor;

    private final PIDController mBottomFlywheelPID = new PIDController(0, 0, 0);
    private final PIDController mTopFlywheelPID = new PIDController(0, 0, 0);

    public FlywheelIOTalonFX() {
        mBottomFlywheel = new TalonFX(KFlywheel.BOTTOM_FLYWHEEL_ID, Constants.CANIVORE_NAME);
        mTopFlywheel = new TalonFX(KFlywheel.TOP_FLYWHEEL_ID, Constants.CANIVORE_NAME);
        mFeederMotor = new TalonFX(KFlywheel.FEEDER_ID, Constants.CANIVORE_NAME);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.bottomVelocityRPM = getBottomFlywheelVelocity();
        inputs.topVelocityRPM = getTopFlywheelVelocity();
        inputs.appliedVolts = mBottomFlywheel.getMotorVoltage().getValueAsDouble() + mTopFlywheel.getMotorVoltage().getValueAsDouble();
    }

    private double getBottomFlywheelVelocity() {
        return mBottomFlywheel.getVelocity().getValueAsDouble() * 60;
    }

    private double getTopFlywheelVelocity() {
        return mTopFlywheel.getVelocity().getValueAsDouble() * 60;
    }

    @Override
    public void setVoltage(double topFlywheelVolts, double bottomFlywheelVolts) {
        mBottomFlywheel.setVoltage(bottomFlywheelVolts);
        mTopFlywheel.setVoltage(topFlywheelVolts);
    }

    @Override
    public void setVelocity(double velocityRPM, double topFfVolts, double bottomFfVolts) {
        mFeederMotor.set(1);

        mBottomFlywheel.setVoltage(mBottomFlywheelPID.calculate(getBottomFlywheelVelocity(), velocityRPM) + bottomFfVolts);
        mTopFlywheel.setVoltage(mTopFlywheelPID.calculate(getBottomFlywheelVelocity(), velocityRPM) + topFfVolts);
    }

    @Override
    public void stop() {
        mBottomFlywheel.stopMotor();
        mTopFlywheel.stopMotor();
        mFeederMotor.stopMotor();
    }

    @Override
    public void configureBottomFlywheelPID(double kP, double kI, double kD) {
        mBottomFlywheelPID.setPID(kP, kI, kD);
    }

    @Override
    public void configureTopFlywheelPID(double kP, double kI, double kD) {
        mTopFlywheelPID.setPID(kP, kI, kD);
    }
}
