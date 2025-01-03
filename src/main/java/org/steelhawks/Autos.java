package org.steelhawks;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.steelhawks.Constants.*;
import org.steelhawks.commands.swerve.DriveCommands;

import java.util.Map;

public final class Autos {

    /** Add your NamedCommands here */
    private static final Map<String, Command> mNamedCommands = Map.of(
        "intake", Commands.print("Intaking"),
        "shoot", Commands.print("Shooting")
    );

    /* Change to the amount of autons we have */
    private static final DigitalInput[] mAutonSelector = {
        new DigitalInput(SelectorConstants.PORT_01),
        new DigitalInput(SelectorConstants.PORT_02),
        new DigitalInput(SelectorConstants.PORT_03),
    };

    static {
        NamedCommands.registerCommands(mNamedCommands);
    }

    private enum AutonMode {
        /* Add your Autons here */
        AUTON_01("test auton", true),
        AUTON_02("test auton 2", false),
        AUTON_03("disabled auton", false);

        private final String autonName;
        private final boolean useVision;
        private Command autonCommand;

        AutonMode(String autonName, boolean useVision) {
            this.autonName = autonName;
            this.useVision = useVision;
        }

        private Command getCommand() {
            if (autonCommand == null) {
                autonCommand = this == AUTON_03 ? Commands.idle() : new PathPlannerAuto(autonName);
            }

            return autonCommand;
        }

        private boolean getUseVision() {
            return useVision;
        }

        private String getAutonName() {
            return autonName;
        }

        public static String getAutonName(int autonSelector) {
            AutonMode[] values = AutonMode.values();

            if (autonSelector >= 0 && autonSelector < values.length) {
                return values[autonSelector].getAutonName();
            }

            return "";
        }

        public static boolean getUseVision(int autonSelector) {
            AutonMode[] values = AutonMode.values();

            if (autonSelector >= 0 && autonSelector < values.length) {
                return values[autonSelector].getUseVision();
            }

            return false;
        }

        public static Command getCommand(int autonSelector) {
            AutonMode[] values = AutonMode.values();

            if (autonSelector >= 0 && autonSelector < values.length) {
                return values[autonSelector].getCommand();
            }

            return null;
        }
    }

    private static int getSelector() {
        for (int i = 0; i < mAutonSelector.length; i++) {
            if (!mAutonSelector[i].get()) {
                return i;
            }
        }

        return -1;
    }

    public static Command getAutonomousCommand() {
        return AutonMode.getCommand(getSelector());
    }

    public static boolean getUseVision() {
        return AutonMode.getUseVision(getSelector());
    }

    public static String getAutonName() {
        return AutonMode.getAutonName(getSelector());
    }

    public static Command testPathfinder() {
        return Commands.runOnce(() -> RobotContainer.s_Swerve.setPose(
            new Pose2d(new Translation2d(1.896013, 5.615929), new Rotation2d())))
            .andThen(
                DriveCommands.driveToPosition(NotePose.NOTE_02)
            );

    }
}
