package org.steelhawks.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.steelhawks.subsystems.intake.Intake.IntakeDir;
import org.steelhawks.subsystems.intake.Intake.IntakeStatus;

public interface IntakeIO {

    @AutoLog
    class IntakeIOInputs {
        public boolean intakeBeamBroken = false;
        public boolean armBeamBroken = false;
        public IntakeDir intakeDir = IntakeDir.NOTHING;
        public IntakeStatus intakeStatus = Intake.IntakeStatus.NOTHING;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void runIntake(IntakeDir dir) {}
    default void stopIntake() {}
}
