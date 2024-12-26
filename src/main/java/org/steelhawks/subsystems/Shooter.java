package org.steelhawks.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.lib.TunableNumber;

/** Example Subsystem */
public class Shooter extends SubsystemBase {

    private final TalonFX mPivot = new TalonFX(0, "canivore");
    private final TalonFX mTopShooter = new TalonFX(2, "canivore");
    private final TalonFX mBottomShooter = new TalonFX(3, "canivore");
    private final CANcoder mPivotEncoder = new CANcoder(1, "canivore");

    private final ProfiledPIDController mPivotController;
    private final ArmFeedforward mPivotFeedforward;

    private final PIDController mBottomShooterController;
    private final SimpleMotorFeedforward mBottomShooterFeedforward;

    private final PIDController mTopShooterController;
    private final SimpleMotorFeedforward mTopShooterFeedforward;

    /* Constants.tuningMode must be true for this to update */
    private final TunableNumber pivotkP = new TunableNumber("shooter/pivotkP", 0);
    private final TunableNumber pivotkI = new TunableNumber("shooter/pivotkI", 0);
    private final TunableNumber pivotkD = new TunableNumber("shooter/pivotkD", 0);
    private final TunableNumber pivotMaxVelocity = new TunableNumber("shooter/pivotMaxVelocity", 0);
    private final TunableNumber pivotMaxAcceleration = new TunableNumber("shooter/pivotMaxAcceleration", 0);

    /* Flywheels */
    private final TunableNumber topShooterkP = new TunableNumber("shooter/topShooterkP", 0);
    private final TunableNumber topShooterkI = new TunableNumber("shooter/topShooterkI", 0);
    private final TunableNumber topShooterkD = new TunableNumber("shooter/topShooterkD", 0);

    private final TunableNumber bottomShooterkP = new TunableNumber("shooter/bottomShooterkP", 0);
    private final TunableNumber bottomShooterkI = new TunableNumber("shooter/bottomShooterkI", 0);
    private final TunableNumber bottomShooterkD = new TunableNumber("shooter/bottomShooterkD", 0);

    private boolean mEnabled = true;
    public void enable() {
        mEnabled = true;
    }
    public void disable() {
        mEnabled = false;
    }
    private boolean isFiring = false;

    private final static Shooter INSTANCE = new Shooter();

    public static Shooter getInstance() {
        return INSTANCE;
    }

    private Shooter() {

        mPivotController = new ProfiledPIDController(pivotkP.get(), pivotkI.get(), pivotkD.get(),
            new TrapezoidProfile.Constraints(pivotMaxVelocity.get(), pivotMaxAcceleration.get()));

        mPivotFeedforward = new ArmFeedforward(0, 0, 0);

        mTopShooterController = new PIDController(topShooterkP.get(), topShooterkI.get(), topShooterkD.get());
        mTopShooterFeedforward = new SimpleMotorFeedforward(0, 0);
        mBottomShooterController = new PIDController(bottomShooterkP.get(), bottomShooterkI.get(), bottomShooterkD.get());
        mBottomShooterFeedforward = new SimpleMotorFeedforward(0, 0);

        mTopShooter.setNeutralMode(NeutralModeValue.Coast);
        mBottomShooter.setNeutralMode(NeutralModeValue.Coast);
        mPivot.setNeutralMode(NeutralModeValue.Brake);
    }

    private void pivot(double output, TrapezoidProfile.State setpoint) {
        double ff = mPivotFeedforward.calculate(setpoint.position, setpoint.velocity);
        mPivot.setVoltage(output + ff);
    }

    private void controlPivotManual(Boolean pivotUp) {
        double pivotkS = .0225;
        double pivotkG = 0.15116;
        double output = Math.cos(getPivotAngle()) * pivotkG;
        double manualVoltage = .75;

        if (pivotUp == null) {
            mPivot.setVoltage(output);
            return;
        }

        double adjustment = pivotkS + manualVoltage;
        mPivot.setVoltage(output + (pivotUp ? adjustment : -adjustment));
    }

    private void setPivotAngle(double radians) {
        mPivotController.setGoal(radians);
    }

    private void rampShooter(double topRPM, double bottomRPM) {
        isFiring = true;

        double topPID = mTopShooterController.calculate(getTopShooterRPM(), topRPM);
        double topFF = mTopShooterFeedforward.calculate(topRPM);

        double bottomPID = mBottomShooterController.calculate(getBottomShooterRPM(), bottomRPM);
        double bottomFF = mBottomShooterFeedforward.calculate(bottomRPM);

        mTopShooter.setVoltage(topPID + topFF);
        mBottomShooter.setVoltage(bottomPID + bottomFF);
    }

    private double getTopShooterRPM() {
        return mTopShooter.getVelocity().getValueAsDouble() * 60;
    }

    private double getBottomShooterRPM() {
        return mBottomShooter.getVelocity().getValueAsDouble() * 60;
    }

    private boolean getTopShooterAtSetpoint() {
        return Math.abs(getTopShooterRPM() - mTopShooterController.getSetpoint()) <= 75;
    }

    private boolean getBottomShooterAtSetpoint() {
        return Math.abs(getBottomShooterRPM() - mBottomShooterController.getSetpoint()) <= 75;
    }

    private boolean getPivotAtSetpoint() {
        return Math.abs(getPivotAngle() - mPivotController.getGoal().position) <= 0.015;
    }

    public Trigger t_IsReadyToShoot() {
        return new Trigger(() -> getTopShooterAtSetpoint() && getBottomShooterAtSetpoint() && getPivotAtSetpoint() && isFiring);
    }

    public double getPivotAngle() {
        return mPivotEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void stopPivot() {
        mPivot.stopMotor();
    }

    public void stopFlywheels() {
        isFiring = false;
        mTopShooter.stopMotor();
        mBottomShooter.stopMotor();
    }

    @Override
    public void periodic() {
        if (pivotkP.hasChanged() ||
                pivotkI.hasChanged() ||
                pivotkD.hasChanged() ||
                pivotMaxVelocity.hasChanged() ||
                pivotMaxAcceleration.hasChanged() ||
                topShooterkP.hasChanged() ||
                topShooterkI.hasChanged() ||
                topShooterkD.hasChanged() ||
                bottomShooterkP.hasChanged() ||
                bottomShooterkI.hasChanged() ||
                bottomShooterkD.hasChanged()) {
            mPivotController.setPID(pivotkP.get(), pivotkI.get(), pivotkD.get());
            mPivotController.setConstraints(new TrapezoidProfile.Constraints(pivotMaxVelocity.get(), pivotMaxAcceleration.get()));

            mTopShooterController.setPID(topShooterkP.get(), topShooterkI.get(), topShooterkD.get());
            mBottomShooterController.setPID(bottomShooterkP.get(), bottomShooterkI.get(), bottomShooterkD.get());
        }

        if (mEnabled) {
            pivot(mPivotController.calculate(getPivotAngle()), mPivotController.getSetpoint());
        }
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////

    public Command getPivotManual(Boolean isPivotingUp) {
        return Commands.sequence(
            Commands.runOnce(this::disable),
            Commands.runEnd(() -> controlPivotManual(isPivotingUp), this::stopPivot, this));
    }

    public Command getHomeCommand() {
        return Commands.run(() -> setPivotAngle(1.04), this);
    }

    public Command getSubwooferShot() {
        return Commands.runEnd(() -> {
            setPivotAngle(1.5);
            rampShooter(1000, 1000);
        }, this::stopFlywheels, this);
    }

    public Command getPodiumShot() {
        return Commands.runEnd(() -> {
            setPivotAngle(1);
            rampShooter(1500, 1500);
        }, this::stopFlywheels, this);
    }
}
