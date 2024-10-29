package org.steelhawks;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {

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

    /* Change to the amount of autons we have */
    private final DigitalInput[] kAutonSelector = {
        new DigitalInput(Constants.SelectorConstants.AUTON_PORT_1),
        new DigitalInput(Constants.SelectorConstants.AUTON_PORT_2),
        new DigitalInput(Constants.SelectorConstants.AUTON_PORT_3),
    };

    private int getSelector() {
        for (int i = 0; i < kAutonSelector.length; i++) {
            if (!kAutonSelector[i].get()) {
                return i;
            }
        }

        return -1;
    }


    public Autos() {
        configureNamedCommands();
    }

    public Command getAutonomousCommand() {
        return AutonMode.getCommand(getSelector());
    }

    public boolean getUseVision() {
        return AutonMode.getUseVision(getSelector());
    }

    public String getAutonName() {
        return AutonMode.getAutonName(getSelector());
    }

    /* Add all NamedCommands used by Pathplanner here */
    private void configureNamedCommands() {}
}
