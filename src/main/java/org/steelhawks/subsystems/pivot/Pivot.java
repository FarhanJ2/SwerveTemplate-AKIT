package org.steelhawks.subsystems.pivot;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.steelhawks.Constants;
import org.steelhawks.OperatorLock;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Pivot extends SubsystemBase {

    private final ProfiledPIDController mPivotController;
    private final ArmFeedforward mPivotFeedforward;

    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private final PivotIO io;
    private OperatorLock mPivotLock = OperatorLock.LOCKED;

    private boolean mEnabled = false;

    public void enable() {
        mEnabled = true;
    }

    public void disable() {
        mEnabled = false;
    }

    public void setDefault(Command defaultCommand) {
        if (getDefaultCommand() != null) {
            getDefaultCommand().cancel();
            removeDefaultCommand();
        }

        setDefaultCommand(defaultCommand);
    }


    public Pivot(PivotIO io) {

        this.io = io;

        switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY -> {
                mPivotController
                    = new ProfiledPIDController(KPivot.PIVOT_KP, KPivot.PIVOT_KI, KPivot.PIVOT_KD,
                        new TrapezoidProfile.Constraints(KPivot.PIVOT_MAX_VELOCITY, KPivot.PIVOT_MAX_ACCELERATION));

                mPivotFeedforward = new ArmFeedforward(KPivot.PIVOT_KS, KPivot.PIVOT_KG, KPivot.PIVOT_KV);
            }
            case SIM -> {
                mPivotController
                    = new ProfiledPIDController(KPivot.PIVOT_KP, KPivot.PIVOT_KI, KPivot.PIVOT_KD,
                        new TrapezoidProfile.Constraints(KPivot.PIVOT_MAX_VELOCITY, KPivot.PIVOT_MAX_ACCELERATION));

                mPivotFeedforward = new ArmFeedforward(0, 0.05, 0.05);
            }
            default -> {
                mPivotController
                    = new ProfiledPIDController(0, 0, 0,
                        new TrapezoidProfile.Constraints(0, 0));

                mPivotFeedforward = new ArmFeedforward(0, 0, 0);
            }
        }

//        setDefault(setPivotHome());

        enable();
    }

    public void setDesiredState(double goal) {
        mPivotController.setGoal(goal);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        if (!mEnabled) return;
        double fb = mPivotController.calculate(inputs.pivotPositionRads);
        double ff = mPivotFeedforward.calculate(mPivotController.getSetpoint().position, mPivotController.getSetpoint().velocity);
        io.runPivot(fb + ff);
    }


    public Command toggleManual(DoubleSupplier leftYAxis) {
        return Commands.either(
            Commands.runOnce(() -> {
                mPivotLock = OperatorLock.LOCKED;
                disable();
                setDefault(
                    runPivotManual(leftYAxis)
                );
            }),
            Commands.runOnce(() -> {
                mPivotLock = OperatorLock.UNLOCKED;
                enable();
                setDefault(
                    setPivotHome()
                );
            }), () -> mPivotLock == OperatorLock.UNLOCKED);
    }


    public Command runPivotManual(DoubleSupplier joystickValue) {
        return Commands.run(
            () -> io.runPivotManual(joystickValue.getAsDouble())
        );
    }

    public Command setPivotHome() {
        return Commands.run(() -> setDesiredState(1.05), this);
    }


}

