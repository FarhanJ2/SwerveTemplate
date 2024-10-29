package org.steelhawks.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class VibrateController extends InstantCommand {


    private static final double DEFAULT_VIBRATE_TIME = 1;
    private final Timer timer = new Timer();

    private final CommandXboxController controller;
    private final double intensity, seconds;


    /**
     * Constructs a command that rumbles the controller.
     *
     * @param controller the controller to vibrate
     * @param intensity the intensity to rumble the controller at between 0.0 and 1.0
     * @param seconds the amount of time to vibrate the controller for
     */
    public VibrateController(CommandXboxController controller, double intensity, double seconds) {

        this.controller = controller;
        this.intensity = intensity;
        this.seconds = seconds;
    }

    /**
     * Constructs a command that rumbles the controller at a set intensity for 1 second.
     *
     * @param controller the controller to vibrate
     * @param intensity the intensity to rumble the controller at between 0.0 and 1.0
     */
    public VibrateController(CommandXboxController controller, double intensity) {
        this(controller, intensity, DEFAULT_VIBRATE_TIME);
    }

    /**
     * Constructs a command that rumbles the controller at full intensity for 1 second.
     *
     * @param controller the controller to vibrate
     */
    public VibrateController(CommandXboxController controller) {
        this(controller, 1.0, DEFAULT_VIBRATE_TIME);
    }

    @Override
    public void initialize() {
        controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, intensity);
        timer.restart();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();

        controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    }
}
