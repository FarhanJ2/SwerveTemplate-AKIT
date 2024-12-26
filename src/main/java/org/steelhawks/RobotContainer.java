// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
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
import org.steelhawks.subsystems.swerve.*;


public class RobotContainer {

    public static boolean addVisionMeasurement = false;
    private static Alliance alliance;

    private RobotMode robotMode = RobotMode.NORMAL_MODE;
    private final Trigger isNormalMode = new Trigger(() -> robotMode == RobotMode.NORMAL_MODE);
    private final Trigger isTestMode = new Trigger(() -> Robot.getState() == Robot.RobotState.TEST && robotMode == RobotMode.NORMAL_MODE);

    /* Subsystems */
    /**
     * Do not delete any of these, or they won't be instantiated even if they are unused
     */
    private final Autos s_Autos = Autos.getInstance();
    public static Swerve s_Swerve;
    public static Intake s_Intake;
    public static Flywheel s_Flywheel;
    private final LED s_LED = LED.getInstance();

    private final CommandXboxController driver = new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator = new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

    /* Button Bindings for Driver */
    private final Trigger bResetGyro = driver.b();
    private final Trigger bToggleVisionMeasurement = driver.povLeft();
    private final Trigger bToggleSpeedMultiplier = driver.rightTrigger();

    /* Button Bindings for Operator */
    private final Trigger bToggleNormalMode = operator.start().and(operator.back());

    private final LoggedDashboardChooser<Command> autoChooser;

    private boolean mRan = false;

    public void waitForDS() {
//        while (!DriverStation.isDSAttached() && !mRan) { // make sure this does not run outside its thread
//            DriverStation.reportWarning("Attaching to the Driver Station...", false);
//        }
//
//        DriverStation.reportWarning("Driver Station Attached", false);

        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        s_LED.setDefaultLighting(s_LED.bounceWaveCommand(alliance == Alliance.Red ? LED.LEDColor.RED : LED.LEDColor.BLUE));

        if (!mRan) {
//            s_Swerve.initializePoseEstimator();

            /* Does nothing, just gets the library ready */
            PathfindingCommand.warmupCommand()
                .finallyDo(() -> s_Swerve.setPose(alliance == Alliance.Blue ? Pose.Blue.ORIGIN : Pose.Red.ORIGIN)) // add this to reset to origin as warmup moves robot pose
                .schedule();

            configurePathfindingCommands();
        }

        s_Swerve.setPose(alliance == Alliance.Red ? Pose.Red.ORIGIN : Pose.Blue.ORIGIN);
        mRan = true;
    }

    public RobotContainer() {
//        new Thread(this::waitForDS).start();

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
            }
            case SIM -> {
                s_Swerve =
                    new Swerve(
                        new GyroIO() {},
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim(),
                        new ModuleIOSim());
                s_Intake
                    = new Intake(new IntakeIOSim());
                s_Flywheel =
                    new Flywheel(new FlywheelIOSim());
            }

            default -> {
                s_Swerve =
                    new Swerve(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});
                s_Intake
                    = new Intake(new IntakeIO() {});
                s_Flywheel =
                    new Flywheel(new FlywheelIO() {});
            }
        }

        configureDefaultCommands();
        configureTestCommands();
        configureAltBindings();
        configureOperator();
        configureTriggers();
        configureDriver();


        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Top Flywheel SysId (Quasistatic Forward)", s_Flywheel.runSysIdQuasistaticTop(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Top Flywheel SysId (Quasistatic Reverse)", s_Flywheel.runSysIdQuasistaticTop(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Bottom Flywheel SysId (Quasistatic Forward)", s_Flywheel.runSysIdQuasistaticBottom(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Bottom Flywheel SysId (Quasistatic Reverse)", s_Flywheel.runSysIdQuasistaticBottom(SysIdRoutine.Direction.kReverse));
    }

    private void configurePathfindingCommands() {
    }

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
    }

    private void configureAltBindings() {
    }

    /* Bindings */
    private void configureDriver() {
        bToggleSpeedMultiplier.onTrue(s_Swerve.toggleMultiplier().alongWith(s_LED.flashCommand(s_Swerve.isSlowMode() ? LED.LEDColor.RED : LED.LEDColor.GREEN, .2, 1)));
        bToggleVisionMeasurement.onTrue(Commands.runOnce(() -> addVisionMeasurement = !addVisionMeasurement));
//        bResetGyro.onTrue(s_Swerve.zeroHeading());

        driver.leftBumper().whileTrue(s_Flywheel.rampSubwoofer());
    }

    private void configureOperator() {
        bToggleNormalMode.onTrue(
            Commands.either(
                Commands.runOnce(() -> robotMode = RobotMode.ALT_MODE),
                Commands.runOnce(() -> robotMode = RobotMode.NORMAL_MODE), isNormalMode)
        );
    }

    private void configureTriggers() {
    }

    private void configureDefaultCommands() {
        s_Swerve.setDefaultCommand(
            DriveCommands.joystickDrive(
                s_Swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                driver.leftTrigger(.5)).withName("Teleop Drive"));
    }

    public static Alliance getAlliance() {
        return alliance;
    }
}
