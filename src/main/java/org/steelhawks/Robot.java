// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.steelhawks.lib.Alert.AlertType;
import org.steelhawks.lib.Alert;
import org.steelhawks.lib.LocalADStarAK;
import org.steelhawks.subsystems.LED;

public class Robot extends LoggedRobot {

    private final RobotContainer mRobotContainer;
    private Command autonomousCommand;
    private Command lightingCommand;

    public enum RobotState {
        DISABLED, TELEOP, AUTON, TEST
    }

    private static RobotState mState = RobotState.DISABLED;

    private void setState(RobotState state) {
        mState = state;
        SmartDashboard.putString("robot/state", state.toString());
    }

    public static RobotState getState() {
        return mState;
    }

    public Robot() {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version);
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_AdvantageKit, 0, WPILibVersion.Version);
        DriverStation.silenceJoystickConnectionWarning(true);
        Logger.recordMetadata("Robot", Constants.ROBOT_NAME); // Set a metadata value

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
                new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
            }
            case SIM -> // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
            case REPLAY -> {
                // Replaying a log, set up replay source
                try {
                    setUseTiming(false); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                    Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                } catch (Exception e) {
                    DriverStation.reportWarning("No replay log found", false);
                }
            }
        }

//        Logger.disableDeterministicTimestamps(); // See "Deterministic Timestamps" in the "Understanding Data Flow" page
        Logger.start();

        // Initialize Robot Container
        mRobotContainer = new RobotContainer();

        if (Constants.TUNING_MODE) {
            new Alert("Tuning Mode Enabled", AlertType.WARNING);
        }
    }

    int counter = 0;

    @Override
    public void robotPeriodic() {
        counter = (counter + 1) % 1000;

        CommandScheduler.getInstance().run();

        /* Update SmartDashboard every 5 cycles (10 time a second) */
        if (counter % 5 == 0) {
            SmartDashboard.putString("auton/auton selected", Autos.getAutonName());
        }
    }


    @Override
    public void disabledInit() {
        setState(RobotState.DISABLED);

        lightingCommand = LED.getInstance().getRainbowCommand();
        lightingCommand.schedule();
    }


    @Override
    public void disabledPeriodic() {
        LED.getInstance().rainbow();
    }


    @Override
    public void disabledExit() {
        if (DriverStation.isDSAttached()) {
            mRobotContainer.waitForDS();
        }
    }

    @Override
    public void autonomousInit() {
//        autonomousCommand = Autos.getAutonomousCommand();
        autonomousCommand = Autos.testPathfinder();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        setState(RobotState.AUTON);
    }


    @Override
    public void autonomousPeriodic() {
    }


    @Override
    public void autonomousExit() {
    }


    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        setState(RobotState.TELEOP);
    }


    @Override
    public void teleopPeriodic() {
    }


    @Override
    public void teleopExit() {
    }


    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        setState(RobotState.TEST);
    }


    @Override
    public void testPeriodic() {
    }


    @Override
    public void testExit() {
    }
}
