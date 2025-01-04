// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.steelhawks.Constants.*;
import org.steelhawks.commands.swerve.DriveCommands;
import org.steelhawks.lib.AllianceFlip;
import org.steelhawks.subsystems.*;
import org.steelhawks.subsystems.flywheel.Flywheel;
import org.steelhawks.subsystems.flywheel.FlywheelIO;
import org.steelhawks.subsystems.flywheel.FlywheelIOSim;
import org.steelhawks.subsystems.flywheel.FlywheelIOTalonFX;
import org.steelhawks.subsystems.intake.Intake;
import org.steelhawks.subsystems.intake.IntakeIO;
import org.steelhawks.subsystems.intake.IntakeIOSim;
import org.steelhawks.subsystems.intake.IntakeIOTalonFX;
import org.steelhawks.subsystems.pivot.Pivot;
import org.steelhawks.subsystems.pivot.PivotIO;
import org.steelhawks.subsystems.pivot.PivotIOSim;
import org.steelhawks.subsystems.pivot.PivotIOTalonFX;
import org.steelhawks.subsystems.swerve.*;


public class RobotContainer {

    public static boolean addVisionMeasurement = false;

    private RobotMode robotMode = RobotMode.NORMAL_MODE;
    private final Trigger isNormalMode = new Trigger(() -> robotMode == RobotMode.NORMAL_MODE);
    private final Trigger isTestMode = new Trigger(() -> Robot.getState() == Robot.RobotState.TEST && robotMode == RobotMode.NORMAL_MODE);

    /* Subsystems */
    public static Swerve s_Swerve;
    public static Intake s_Intake;
    public static Flywheel s_Flywheel;
    public static Pivot s_Pivot;
    private final LED s_LED = LED.getInstance();

    private final CommandXboxController driver = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    /* Button Bindings for Driver */
    private final Trigger bResetGyro = driver.b();
    private final Trigger bToggleVisionMeasurement = driver.povLeft();
    private final Trigger bToggleSpeedMultiplier = driver.rightTrigger();

    /* Button Bindings for Operator */
    private final Trigger bToggleNormalMode = operator.start().and(operator.back());

    private boolean mRan = false;

    public void waitForDS() {
        boolean isRed =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;


        s_LED.setDefaultLighting(s_LED.bounceWaveCommand(isRed ? LED.LEDColor.RED : LED.LEDColor.BLUE));

        if (mRan) return;

        mRan = true;
    }

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                s_Swerve =
                    new Swerve(
                        new GyroIOPigeon2(true),
                        new ModuleIOTalonFX(0),
                        new ModuleIOTalonFX(1),
                        new ModuleIOTalonFX(2),
                        new ModuleIOTalonFX(3));
                s_Intake =
                    new Intake(new IntakeIOTalonFX());
                s_Flywheel =
                    new Flywheel(new FlywheelIOTalonFX());
                s_Pivot =
                    new Pivot(new PivotIOTalonFX());
            }
            case SIM -> {
                s_Swerve =
                    new Swerve(
                        new GyroIO() {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                s_Intake =
                    new Intake(new IntakeIOSim());
                s_Flywheel =
                    new Flywheel(new FlywheelIOSim());
                s_Pivot =
                    new Pivot(new PivotIOSim());
            }
            default -> {
                s_Swerve =
                    new Swerve(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                s_Intake =
                    new Intake(new IntakeIO() {});
                s_Flywheel =
                    new Flywheel(new FlywheelIO() {});
                s_Pivot =
                    new Pivot(new PivotIO() {});
            }
        }

        configurePathfindingCommands();
        configureDefaultCommands();
        configureSysIdBindings();
        configureTestCommands();
        configureAltBindings();
        configureOperator();
        configureTriggers();
        configureDriver();
    }

    private void configureSysIdBindings() {
        LoggedDashboardChooser<Command> mSysIdSelector =
            new LoggedDashboardChooser<>("SysId Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        mSysIdSelector.addOption(
            "Drive SysId (Quasistatic Forward)",
            s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        mSysIdSelector.addOption(
            "Drive SysId (Quasistatic Reverse)",
            s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        mSysIdSelector.addOption(
            "Drive SysId (Dynamic Forward)", s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        mSysIdSelector.addOption(
            "Drive SysId (Dynamic Reverse)", s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        mSysIdSelector.addOption(
            "Top Flywheel SysId (Quasistatic Forward)", s_Flywheel.runSysIdQuasistaticTop(SysIdRoutine.Direction.kForward));
        mSysIdSelector.addOption(
            "Top Flywheel SysId (Quasistatic Reverse)", s_Flywheel.runSysIdQuasistaticTop(SysIdRoutine.Direction.kReverse));
        mSysIdSelector.addOption(
            "Bottom Flywheel SysId (Quasistatic Forward)", s_Flywheel.runSysIdQuasistaticBottom(SysIdRoutine.Direction.kForward));
        mSysIdSelector.addOption(
            "Bottom Flywheel SysId (Quasistatic Reverse)", s_Flywheel.runSysIdQuasistaticBottom(SysIdRoutine.Direction.kReverse));
    }

    private void configurePathfindingCommands() {
        driver.povLeft()
            .onTrue(
                DriveCommands.driveToPosition(NotePose.NOTE_02,
                        () -> Math.abs(driver.getLeftY()
                        + driver.getLeftX()
                        + driver.getRightX()) > 0.1));

        driver.povRight()
            .onTrue(
                Commands.runOnce(() ->
                    s_Swerve.setPose(
                        new Pose2d(new Translation2d(1.472591, 5.563001), new Rotation2d()))));

    }

    SendableChooser<Pose2d> chooser = new SendableChooser<>();

    private void configureTestCommands() {
        /* Sample Test Mode */
        driver.leftStick()
            .and(driver.rightStick())
            .and(isTestMode)
            .onTrue(
                Commands.sequence(
                    Commands.print("Testing Intake"),
                    Commands.print("Testing Shooter"),
                    Commands.print("Testing Elevator"),
                    Commands.print("Done Testing!")
                ).withName("Pit Test"));



        chooser.addOption("Speaker", AllianceFlip.validate(FieldConstants.BLUE_SPEAKER_POSE));
        chooser.addOption("NOTE 1", AllianceFlip.validate(NotePose.NOTE_01));
        chooser.addOption("NOTE 2", AllianceFlip.validate(NotePose.NOTE_02));
        chooser.addOption("NOTE 3", AllianceFlip.validate(NotePose.NOTE_03));
        chooser.addOption("FAR NOTE 1", AllianceFlip.validate(NotePose.FAR_NOTE_01));
        chooser.addOption("FAR NOTE 2", AllianceFlip.validate(NotePose.FAR_NOTE_02));
        chooser.addOption("FAR NOTE 3", AllianceFlip.validate(NotePose.FAR_NOTE_03));
        chooser.addOption("FAR NOTE 4", AllianceFlip.validate(NotePose.FAR_NOTE_04));
        chooser.addOption("FAR NOTE 5", AllianceFlip.validate(NotePose.FAR_NOTE_05));

        chooser.setDefaultOption("FAR NOTE 5", AllianceFlip.validate(NotePose.FAR_NOTE_05));

        SmartDashboard.putData(chooser);
    }

    private void configureAltBindings() {}

    /* Bindings */
    private void configureDriver() {
        bToggleSpeedMultiplier.onTrue(s_Swerve.toggleMultiplier().alongWith(s_LED.flashCommand(s_Swerve.isSlowMode() ? LED.LEDColor.RED : LED.LEDColor.GREEN, .2, 1)));
        bToggleVisionMeasurement.onTrue(Commands.runOnce(() -> addVisionMeasurement = !addVisionMeasurement));
        bResetGyro.onTrue(s_Swerve.zeroHeading());

//        driver.leftBumper().whileTrue(s_Flywheel.rampSubwoofer());

        driver.rightBumper().whileTrue(
            DriveCommands.rotateToAngle(FieldConstants.BLUE_SPEAKER_POSE));

        driver.x()
            .onTrue(Commands.runOnce(s_Swerve::stopWithX));

        driver.leftBumper()
            .onTrue(
                DriveCommands.driveToPosition(
                    chooser.getSelected() != null ?
                        chooser.getSelected() : new Pose2d(),
                            () -> Math.abs(driver.getLeftY()
                                + driver.getLeftX()
                                    + driver.getRightX()) > 0.1));
    }

    private void configureOperator() {
        bToggleNormalMode.onTrue(
            Commands.either(
                Commands.runOnce(() -> robotMode = RobotMode.ALT_MODE),
                Commands.runOnce(() -> robotMode = RobotMode.NORMAL_MODE), isNormalMode));

        operator.leftStick()
            .onTrue(s_Pivot.toggleManual(operator::getLeftY));
    }

    private void configureTriggers() {}

    private void configureDefaultCommands() {
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                driver.leftTrigger(.5)));
    }
}
