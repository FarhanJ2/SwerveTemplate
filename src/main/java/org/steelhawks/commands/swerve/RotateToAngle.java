package org.steelhawks.commands.swerve;

import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;
import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import static org.steelhawks.lib.MathUtil.continuous180To360;

public class RotateToAngle extends Command {
    private final Supplier<Double> requestedAngle;
    private final Supplier<Boolean> buttonPressed;

    private final PIDController alignPID = new PIDController(
        Constants.Swerve.autoAlignKP,
        Constants.Swerve.autoAlignKI,
        Constants.Swerve.autoAlignKD
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
        double robotHeading = continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees());
        double setpoint = (robotHeading + requestedAngle.get()) % 360;

        alignPID.setSetpoint(setpoint);
    }

    @Override
    public void execute() {
        RobotContainer.s_Swerve.drive(
            new Translation2d(),
            (RobotContainer.s_Swerve.isSlowMode() ? 5 : 1) * alignPID.calculate(continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees())),
            true,
            false
        );
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