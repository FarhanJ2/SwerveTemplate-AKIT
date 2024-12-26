package org.steelhawks.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;
import org.steelhawks.subsystems.swerve.Swerve;

import static org.steelhawks.lib.MathUtil.continuous180To360;


public class DriveToPosition extends Command {

    ///////////////////////////
    /* THIS IS UNTESTED CODE */
    ///////////////////////////

    private final PIDController mXPositionController = new PIDController(Constants.AutonConstants.TRANSLATION_KP, Constants.AutonConstants.TRANSLATION_KI, Constants.AutonConstants.TRANSLATION_KD);
    private final PIDController mYPositionController = new PIDController(Constants.AutonConstants.TRANSLATION_KP, Constants.AutonConstants.TRANSLATION_KI, Constants.AutonConstants.TRANSLATION_KD);
    private final PIDController mAlignController = RobotContainer.s_Swerve.getAlignPID();

    private final Pose2d mTarget;

    public DriveToPosition(Pose2d target) {
        mTarget = target;

        mAlignController.setTolerance(1);

        addRequirements(RobotContainer.s_Swerve);
    }

    @Override
    public void initialize() {
        double robotHeading = continuous180To360(RobotContainer.s_Swerve.getRotation().getDegrees());
        double setpoint = (robotHeading + mTarget.getRotation().getDegrees()) % 360;

        mAlignController.setSetpoint(setpoint);
        mXPositionController.setSetpoint(mTarget.getX());
        mYPositionController.setSetpoint(mTarget.getY());
    }

    @Override
    public void execute() {
//        Swerve.getInstance().drive(
//            new Translation2d(mXPositionController.calculate(Swerve.getInstance().getPose().getX()), mYPositionController.calculate(Swerve.getInstance().getPose().getY())),
//            mAlignController.calculate(continuous180To360(mTarget.getRotation().getDegrees())),
//            true, false
//        );
    }

    @Override
    public boolean isFinished() {
        return mAlignController.atSetpoint() && mXPositionController.atSetpoint() && mYPositionController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Swerve.stop();
    }
}
