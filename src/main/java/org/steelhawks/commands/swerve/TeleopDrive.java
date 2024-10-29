package org.steelhawks.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.steelhawks.lib.MathUtil.continuous180To360;

public class TeleopDrive extends Command {

    private final DoubleSupplier translation;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;
    private final BooleanSupplier fieldRelative;

    public TeleopDrive(DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric) {
        addRequirements(RobotContainer.s_Swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fieldRelative = robotCentric;
    }

    /* Use this for rotating to a pose/structure */
    private double getRotationSpeedFromPID(Pose2d target) {
        double robotHeading = continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees());
        double requestedAngle = RobotContainer.s_Swerve.calculateTurnAngle(target, RobotContainer.s_Swerve.getHeading().getDegrees() + 180);
        double setpoint = (robotHeading + requestedAngle) % 360;

        RobotContainer.s_Swerve.getAlignController().setSetpoint(setpoint);

        return (RobotContainer.s_Swerve.isSlowMode() ? 5 : 1) * RobotContainer.s_Swerve.getAlignController().calculate(continuous180To360(RobotContainer.s_Swerve.getHeading().getDegrees()));
    }

    @Override
    public void execute() {
        double translationValue = MathUtil.applyDeadband(translation.getAsDouble(), Constants.Deadbands.DRIVE_DEADBAND);
        double strafeValue = MathUtil.applyDeadband(strafe.getAsDouble(), Constants.Deadbands.DRIVE_DEADBAND);
        double rotationValue = MathUtil.applyDeadband(rotation.getAsDouble(), Constants.Deadbands.DRIVE_DEADBAND);

        Translation2d multipliedTranslation = new Translation2d(translationValue, strafeValue).times(Constants.Swerve.MAX_SPEED);
        double multipliedRotation = rotationValue * Constants.Swerve.MAX_ANGULAR_VELOCITY;

        RobotContainer.s_Swerve.drive(
            multipliedTranslation,
            multipliedRotation,
            fieldRelative.getAsBoolean(),
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.s_Swerve.stop();
    }
}
