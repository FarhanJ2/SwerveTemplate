// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.commands.swerve.TeleopDrive;
import org.steelhawks.subsystems.LED;
import org.steelhawks.subsystems.Swerve;


public class RobotContainer {
    public static final CTREConfigs configs = new CTREConfigs();
    public static DriverStation.Alliance alliance;

    public static boolean addVisionMeasurement = false;

    public RobotMode robotMode = RobotMode.NORMAL_MODE;
    public Trigger isNormalMode = new Trigger(() -> robotMode == RobotMode.NORMAL_MODE);

    /* Subsystems */
    public static final Autos s_Autos = new Autos();
    public static final Swerve s_Swerve = new Swerve();
    public static final LED s_LED = new LED(Constants.LED.LED_PORT, Constants.LED.LED_STRIP_LENGTH);

    private final CommandXboxController kDriverController = new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController kOperatorController = new CommandXboxController(Constants.OIConstants.OPERATOR_CONTROLLER_PORT);

    /* Button Triggers */
    private final Trigger kResetGyro = kDriverController.b();
    private final Trigger kToggleVisionMeasurement = kDriverController.povLeft();

    public RobotContainer() {
        new Thread(() -> {
            while (!DriverStation.isDSAttached()) {
                DriverStation.reportWarning("Attaching to the Driver Station...", false);
            }

            DriverStation.reportWarning("Driver Station Attached", false);

            if (DriverStation.getAlliance().isPresent()) {
                alliance = DriverStation.getAlliance().get();
            }

            s_Swerve.initializePoseEstimator();
            s_LED.setDefaultCommand(s_LED.setColorCommand(alliance == DriverStation.Alliance.Red ? LED.LEDColor.RED : LED.LEDColor.BLUE));
        }).start();

        configureDefaultCommands();
        configureAltBindings();
        configureOperator();
        configureTriggers();
        configureDriver();
    }

    private void configureAltBindings() {}

    /* Bindings */
    private void configureDriver() {
        kResetGyro.onTrue(Commands.runOnce(s_Swerve::zeroHeading));
        kToggleVisionMeasurement.onTrue(Commands.runOnce(() -> addVisionMeasurement = !addVisionMeasurement));
    }
    private void configureOperator() {}

    private void configureTriggers() {}

    private void configureDefaultCommands() {
        s_Swerve.setDefaultCommand(
            new TeleopDrive(
                    () -> -kDriverController.getLeftY(),
                    () -> -kDriverController.getLeftX(),
                    () -> -kDriverController.getRightX(),
                    () -> true // field relative
            ));
    }

}
