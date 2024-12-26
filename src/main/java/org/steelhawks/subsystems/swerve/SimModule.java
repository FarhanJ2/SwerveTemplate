package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.steelhawks.lib.Conversions;

public class SimModule implements ModuleIO {

    private final DCMotorSim mDriveMotor = new DCMotorSim(DCMotor.getFalcon500(1), 6.75, 0.025);
    private final DCMotorSim mAngleMotor = new DCMotorSim(DCMotor.getFalcon500(1), 150.0 / 7.0, 0.004);

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(KSwerve.driveKS, KSwerve.driveKV, KSwerve.driveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setInputVoltage(anglePosition.withPosition(desiredState.angle.getRotations()).FeedForward);
        setSpeed(desiredState, isOpenLoop);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / KSwerve.MAX_SPEED;
            mDriveMotor.setInputVoltage(driveDutyCycle.Output);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, KSwerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setInputVoltage(driveVelocity.withVelocity(desiredState.speedMetersPerSecond).FeedForward);
        }
    }

    @Override
    public void resetToAbsolute() {}

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getAngularVelocityRPM() / 60, KSwerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(mAngleMotor.getAngularPositionRotations() / 60)
        );
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getAngularPositionRotations(), KSwerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(mAngleMotor.getAngularPositionRotations())
        );
    }

    @Override
    public double getDriveVoltage() {
        return 0;
    }
}
