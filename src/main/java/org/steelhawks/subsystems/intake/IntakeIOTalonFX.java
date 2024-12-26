package org.steelhawks.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.steelhawks.Constants;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX mIntakeMotor1;
    private final TalonFX mIntakeMotor2;
    private final TalonFX mDivertorMotor;

    private Intake.IntakeDir dir;

    public IntakeIOTalonFX() {
        mIntakeMotor1 = new TalonFX(KIntake.INTAKE_ID_1, Constants.CANIVORE_NAME);
        mIntakeMotor2 = new TalonFX(KIntake.INTAKE_ID_2, Constants.CANIVORE_NAME);
        mDivertorMotor = new TalonFX(KIntake.DIVERTOR_ID, Constants.CANIVORE_NAME);

        mIntakeMotor1.setInverted(true);
        mIntakeMotor2.setInverted(false);
        mDivertorMotor.setInverted(true);

        mIntakeMotor1.setNeutralMode(NeutralModeValue.Coast);
        mIntakeMotor2.setNeutralMode(NeutralModeValue.Coast);
        mDivertorMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeDir = dir;
    }

    @Override
    public void runIntake(Intake.IntakeDir dir) {
        this.dir = dir;

        switch (dir) {
            case TO_ARM -> {
                mIntakeMotor1.set(1 / 3.0);
                mIntakeMotor2.set(1 / 3.0);
                mDivertorMotor.set(1 / 3.0);
            }
            case TO_HOLDER -> {
                mIntakeMotor1.set(1);
                mIntakeMotor1.set(1);
                mDivertorMotor.setVoltage(1);
            }
            case TO_SHOOTER -> {
                mIntakeMotor1.set(1);
                mIntakeMotor1.set(1);
                mDivertorMotor.set(1);
            }
            default -> stopIntake();
        }
    }

    @Override
    public void stopIntake() {
        mIntakeMotor1.stopMotor();
        mIntakeMotor2.stopMotor();
        mDivertorMotor.stopMotor();
    }
}
