// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.steelhawks.commands.swerve.TeleopDrive;
import org.steelhawks.subsystems.*;
import org.steelhawks.subsystems.swerve.Swerve;


public class RobotContainer {

    public static final CTREConfigs configs = new CTREConfigs();
    public static boolean addVisionMeasurement = false;
    private static DriverStation.Alliance alliance;

    private RobotMode robotMode = RobotMode.NORMAL_MODE;
    private final Trigger isNormalMode = new Trigger(() -> robotMode == RobotMode.NORMAL_MODE);

    /* Subsystems */
    private final Autos s_Autos = Autos.getInstance();
    private final Swerve s_Swerve = Swerve.getInstance();
    private final LED s_LED = LED.getInstance();

    private final CommandXboxController driver = new CommandXboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator = new CommandXboxController(Constants.OIConstants.OPERATOR_CONTROLLER_PORT);

    /* Button Bindings */
    private final Trigger bResetGyro = driver.b();
    private final Trigger bToggleVisionMeasurement = driver.povLeft();
    private final Trigger bToggleSpeedMultiplier = driver.rightTrigger();

    private final Trigger bToggleNormalMode = operator.start().and(operator.back());

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
            s_LED.setDefaultLighting(s_LED.setColorCommand(alliance == DriverStation.Alliance.Red ? LED.LEDColor.RED : LED.LEDColor.BLUE));
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
        bToggleSpeedMultiplier.onTrue(Commands.runOnce(s_Swerve::toggleMultiplier).alongWith(s_LED.flashCommand(s_Swerve.isSlowMode() ? LED.LEDColor.YELLOW : LED.LEDColor.GREEN, .2, 1)));
        bToggleVisionMeasurement.onTrue(Commands.runOnce(() -> addVisionMeasurement = !addVisionMeasurement));
        bResetGyro.onTrue(Commands.runOnce(s_Swerve::zeroHeading));
    }

    private void configureOperator() {
        bToggleNormalMode.onTrue(
            Commands.either(
                Commands.runOnce(() -> robotMode = RobotMode.ALT_MODE),
                Commands.runOnce(() -> robotMode = RobotMode.NORMAL_MODE), isNormalMode)
        );
    }

    private void configureTriggers() {}

    private void configureDefaultCommands() {
        s_Swerve.setDefaultCommand(
            new TeleopDrive(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> true
            ));
    }

    public static DriverStation.Alliance getAlliance() {
        return alliance;
    }
}
