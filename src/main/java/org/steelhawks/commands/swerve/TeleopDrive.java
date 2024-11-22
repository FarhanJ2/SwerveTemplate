package org.steelhawks.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.Constants;
import org.steelhawks.subsystems.swerve.Swerve;
import org.steelhawks.subsystems.swerve.KSwerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.steelhawks.lib.MathUtil.continuous180To360;

public class TeleopDrive extends Command {

    private final DoubleSupplier translation;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;
    private final BooleanSupplier fieldRelative;

    public TeleopDrive(DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric) {
        addRequirements(Swerve.getInstance());

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fieldRelative = robotCentric;
    }

    /* Use this for rotating to a pose/structure */
    private double getRotationSpeedFromPID(Pose2d target) {
        double robotHeading = continuous180To360(Swerve.getInstance().getHeading().getDegrees());
        double requestedAngle = Swerve.getInstance().calculateTurnAngle(target, Swerve.getInstance().getHeading().getDegrees() + 180);
        double setpoint = (robotHeading + requestedAngle) % 360;

        Swerve.getInstance().getAlignController().setSetpoint(setpoint);

        return (Swerve.getInstance().isSlowMode() ? 5 : 1) * Swerve.getInstance().getAlignController().calculate(continuous180To360(Swerve.getInstance().getHeading().getDegrees()));
    }

    @Override
    public void execute() {
        double translationValue = MathUtil.applyDeadband(translation.getAsDouble(), Constants.Deadbands.DRIVE_DEADBAND);
        double strafeValue = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.Deadbands.DRIVE_DEADBAND);
        double rotationValue = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.Deadbands.DRIVE_DEADBAND);

        Translation2d multipliedTranslation = new Translation2d(translationValue, strafeValue).times(KSwerve.MAX_SPEED);
        double multipliedRotation = rotationValue * KSwerve.MAX_ANGULAR_VELOCITY;

        Swerve.getInstance().drive(
            multipliedTranslation,
            multipliedRotation,
            fieldRelative.getAsBoolean(),
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().stop();
    }
}
