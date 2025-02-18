package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Swerve/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {

    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder canCoder;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Double> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveCurrent;

    private final StatusSignal<Double> turnAbsolutePosition;
    private final StatusSignal<Double> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<Double> turnVelocity;
    private final StatusSignal<Double> turnAppliedVolts;
    private final StatusSignal<Double> turnCurrent;

    // Gear ratios for SDS MK4i L2, adjust as necessary
    private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    private final double TURN_GEAR_RATIO = 150.0 / 7.0;

    private final boolean isTurnMotorInverted = true;
    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(int index) {
        switch (index) {
            case 0: // fl
                driveTalon = new TalonFX(1, Constants.CANIVORE_NAME);
                turnTalon = new TalonFX(2, Constants.CANIVORE_NAME);
                canCoder = new CANcoder(3, Constants.CANIVORE_NAME);
                absoluteEncoderOffset = Rotation2d.fromDegrees(-130.17);
                break;
            case 1: // fr
                driveTalon = new TalonFX(4, Constants.CANIVORE_NAME);
                turnTalon = new TalonFX(5, Constants.CANIVORE_NAME);
                canCoder = new CANcoder(6, Constants.CANIVORE_NAME);
                absoluteEncoderOffset = Rotation2d.fromDegrees(-51.34);
                break;
            case 2: // bl
                driveTalon = new TalonFX(7, Constants.CANIVORE_NAME);
                turnTalon = new TalonFX(8, Constants.CANIVORE_NAME);
                canCoder = new CANcoder(9, Constants.CANIVORE_NAME);
                absoluteEncoderOffset = Rotation2d.fromDegrees(-63.9);
                break;
            case 3: // br
                driveTalon = new TalonFX(10, Constants.CANIVORE_NAME);
                turnTalon = new TalonFX(11, Constants.CANIVORE_NAME);
                canCoder = new CANcoder(12, Constants.CANIVORE_NAME);
                absoluteEncoderOffset = Rotation2d.fromDegrees(-175.96);
                break;
            default:
                throw new RuntimeException("Invalid module index");
        }

        var driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveTalon.getConfigurator().apply(driveConfig);
        setDriveBrakeMode(true);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnTalon.getConfigurator().apply(turnConfig);
        setTurnBrakeMode(true);

        canCoder.getConfigurator().apply(new CANcoderConfiguration());

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getSupplyCurrent();

        turnAbsolutePosition = canCoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue =
            PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            SwerveModule.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent);
        driveTalon.optimizeBusUtilization();
        turnTalon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveCurrent,
            turnAbsolutePosition,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnCurrent);

        inputs.drivePositionRad =
            Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveVelocityRadPerSec =
            Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

        inputs.turnAbsolutePosition =
            Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble())
                .minus(absoluteEncoderOffset);
        inputs.turnPosition =
            Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
        inputs.turnVelocityRadPerSec =
            Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
                .toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted = InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(config);
    }

    @Override
    public void setTurnBrakeMode(boolean enable) {
        var config = new MotorOutputConfigs();
        config.Inverted =
            isTurnMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnTalon.getConfigurator().apply(config);
    }
}
