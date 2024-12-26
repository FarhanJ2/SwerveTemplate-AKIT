package org.steelhawks.subsystems.intake;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;

    private IntakeStatus status = IntakeStatus.NOTHING;

    private final DigitalInput mIntakeBeam = new DigitalInput(KIntake.INTAKE_BEAM_ID);
    private final DigitalInput mArmBeam = new DigitalInput(KIntake.ARM_BEAM_ID);

    public enum IntakeDir {
        NOTHING,
        TO_HOLDER,
        TO_ARM,
        TO_SHOOTER
    }

    public enum IntakeStatus {
        IN_HOLDER,
        IN_ARM,
        IN_SHOOTER,
        NOTHING
    }

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public boolean intakeBeamBroken() {
        return !mIntakeBeam.get();
    }

    public boolean armBeamBroken() {
        return !mArmBeam.get();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        inputs.armBeamBroken = armBeamBroken();
        inputs.intakeBeamBroken = intakeBeamBroken();
        inputs.intakeStatus = status;
    }




    public Command intake() {
        return Commands.runEnd(
            () -> io.runIntake(IntakeDir.TO_HOLDER),
            io::stopIntake
        ).until(this::intakeBeamBroken);
    }
}

