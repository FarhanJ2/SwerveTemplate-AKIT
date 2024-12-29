package org.steelhawks.subsystems.flywheel;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;

import static edu.wpi.first.units.Units.Volts;

public class Flywheel extends SubsystemBase {

    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
    private final SimpleMotorFeedforward bottomFlywheelFeedforward;
    private final SimpleMotorFeedforward topFlywheelFeedforward;
    private final SysIdRoutine sysIdTop, sysIdBottom;
    private final FlywheelIO io;

    public Flywheel(FlywheelIO io) {
        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY -> {
                bottomFlywheelFeedforward = new SimpleMotorFeedforward(KFlywheel.BOTTOM_FLYWHEEL_KS, KFlywheel.BOTTOM_FLYWHEEL_KV, KFlywheel.BOTTOM_FLYWHEEL_KA);
                topFlywheelFeedforward = new SimpleMotorFeedforward(KFlywheel.TOP_FLYWHEEL_KS, KFlywheel.TOP_FLYWHEEL_KV, KFlywheel.TOP_FLYWHEEL_KA);
                io.configureBottomFlywheelPID(KFlywheel.BOTTOM_FLYWHEEL_KP, KFlywheel.BOTTOM_FLYWHEEL_KI, KFlywheel.BOTTOM_FLYWHEEL_KD);
                io.configureTopFlywheelPID(KFlywheel.TOP_FLYWHEEL_KP, KFlywheel.TOP_FLYWHEEL_KI, KFlywheel.TOP_FLYWHEEL_KD);
            }
            case SIM -> {
                bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                topFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
                io.configureBottomFlywheelPID(KFlywheel.BOTTOM_FLYWHEEL_KP, KFlywheel.BOTTOM_FLYWHEEL_KI, KFlywheel.BOTTOM_FLYWHEEL_KD);
                io.configureTopFlywheelPID(KFlywheel.TOP_FLYWHEEL_KP, KFlywheel.TOP_FLYWHEEL_KI, KFlywheel.TOP_FLYWHEEL_KD);
            }
            default -> {
                bottomFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                topFlywheelFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                io.configureBottomFlywheelPID(0.0, 0.0, 0.0);
                io.configureTopFlywheelPID(0.0, 0.0, 0.0);
            }
        }

        sysIdTop =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts), 0), null, this));

        sysIdBottom =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runVolts(0, voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Flywheel", inputs);
    }

    public void runVolts(double topFlywheelVolts, double bottomFlywheelVolts) {
        io.setVoltage(topFlywheelVolts, bottomFlywheelVolts);
    }

    public void stop() {
        io.stop();
    }

    @AutoLogOutput ( key = "Flywheel/Top Velocity RPM")
    public double getTopVelocityRPM() {
        return inputs.topVelocityRPM;
    }

    @AutoLogOutput( key = "Flywheel/Bottom Velocity RPM")
    public double getBottomVelocityRPM() {
        return inputs.bottomVelocityRPM;
    }


    public Command rampSubwoofer() {
        return Commands.runEnd(
            () -> io.setVelocity(1500, topFlywheelFeedforward.calculate(1500), bottomFlywheelFeedforward.calculate(1500)),
            this::stop
        );
    }

    public Command runSysIdQuasistaticTop(SysIdRoutine.Direction dir) {
        return sysIdTop.quasistatic(dir);
    }

    public Command runSysIdQuasistaticBottom(SysIdRoutine.Direction dir) {
        return sysIdBottom.quasistatic(dir);
    }

    public Command runSysIdDynamicTop(SysIdRoutine.Direction dir) {
        return sysIdTop.dynamic(dir);
    }

    public Command runSysIdDynamicBottom(SysIdRoutine.Direction dir) {
        return sysIdBottom.dynamic(dir);
    }

}


