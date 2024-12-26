package org.steelhawks.subsystems.flywheel;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {

    private final FlywheelSim mBottomFlywheel = new FlywheelSim(DCMotor.getFalcon500(1), 1.5, 0.004);
    private final FlywheelSim mTopFlywheel = new FlywheelSim(DCMotor.getFalcon500(1), 1.5, 0.004);

    private final PIDController mBottomFlywheelPID = new PIDController(0, 0, 0);
    private final PIDController mTopFlywheelPID = new PIDController(0, 0, 0);

    private double topFfVolts, bottomFfVolts;

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {

        mBottomFlywheel.setInputVoltage(bottomFfVolts + mBottomFlywheelPID.calculate(mBottomFlywheel.getAngularVelocityRPM()));
        mTopFlywheel.setInputVoltage(topFfVolts + mTopFlywheelPID.calculate(mTopFlywheel.getAngularVelocityRPM()));

        mBottomFlywheel.update(0.02);
        mTopFlywheel.update(0.02);

        inputs.bottomVelocityRPM = mBottomFlywheel.getAngularVelocityRPM();
        inputs.topVelocityRPM = mTopFlywheel.getAngularVelocityRPM();
    }

    @Override
    public void setVoltage(double topFlywheelVolts, double bottomFlywheelVolts) {
        mTopFlywheel.setInputVoltage(topFlywheelVolts);
        mBottomFlywheel.setInputVoltage(bottomFlywheelVolts);
    }

    @Override
    public void setVelocity(double velocityRPM, double topFfVolts, double bottomFfVolts) {
        this.topFfVolts = topFfVolts;
        this.bottomFfVolts = bottomFfVolts;

        mBottomFlywheelPID.setSetpoint(velocityRPM);
        mTopFlywheelPID.setSetpoint(velocityRPM);
    }

    @Override
    public void stop() {
        setVoltage(0, 0);
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
