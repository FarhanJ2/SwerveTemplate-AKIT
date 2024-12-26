package org.steelhawks.commands.swerve;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.swerve.KSwerve;

import static org.steelhawks.lib.MathUtil.continuous180To360;

public class RotateToAngle extends Command {
    private final Supplier<Double> requestedAngle;
    private final Supplier<Boolean> buttonPressed;

    private final PIDController alignPID = new PIDController(
        KSwerve.autoAlignKP,
        KSwerve.autoAlignKI,
        KSwerve.autoAlignKD
    );

    public RotateToAngle(Supplier<Double> requestedAngle) {
        this(requestedAngle, () -> true);
    }

    public RotateToAngle(Supplier<Double> requestedAngle, Supplier<Boolean> buttonPressed) {

        alignPID.enableContinuousInput(0, 360);
        alignPID.setTolerance(1);

        this.buttonPressed = buttonPressed;
        this.requestedAngle = requestedAngle;

        addRequirements(RobotContainer.s_Swerve);
    }

    @Override
    public void initialize() {
        double robotHeading = continuous180To360(RobotContainer.s_Swerve.getRotation().getDegrees());
        double setpoint = (robotHeading + requestedAngle.get()) % 360;

        alignPID.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
//        Swerve.getInstance().drive(
//            new Translation2d(),
//            (Swerve.getInstance().isSlowMode() ? 5 : 1) * alignPID.calculate(continuous180To360(RobotContainer.s_Swerve.getRotation().getDegrees())),
//            true,
//            false
//        );
    }

    @Override
    public boolean isFinished() {
        if (buttonPressed == null) return alignPID.atSetpoint();
        else return (alignPID.atSetpoint() && !buttonPressed.get())
                || !buttonPressed.get();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Swerve.stop();
    }
}
