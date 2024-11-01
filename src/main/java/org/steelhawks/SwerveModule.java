package org.steelhawks;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import org.steelhawks.lib.Conversions;
import org.steelhawks.lib.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private final Rotation2d angleOffset;

    private final TalonFX mAngleMotor;
    private final TalonFX mDriveMotor;
    private final CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.getAngleOffset();

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.getCancoderID(), Constants.CANIVORE_NAME);
        angleEncoder.getConfigurator().apply(RobotContainer.configs.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.getAngleMotorID(), Constants.CANIVORE_NAME);
        mAngleMotor.getConfigurator().apply(RobotContainer.configs.swerveAngleFXConfig);

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.getDriveMotorID(), Constants.CANIVORE_NAME);
        mDriveMotor.getConfigurator().apply(RobotContainer.configs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        configureStatusFrameRates();
    }

    private void configureStatusFrameRates() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                mAngleMotor.getVelocity(),
                mAngleMotor.getPosition(),

                mDriveMotor.getVelocity(),
                mDriveMotor.getPosition(),
                mDriveMotor.getMotorVoltage()
        );

        mAngleMotor.optimizeBusUtilization();
        mDriveMotor.optimizeBusUtilization();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    public double getVoltage() {
        return mDriveMotor.getMotorVoltage().getValueAsDouble();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.WHEEL_CIRCUMFERENCE),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.WHEEL_CIRCUMFERENCE), //mDriveMotor.getPosition().getValue()
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble())
        );
    }

    public TalonFX getDriveMotor() {
        return mDriveMotor;
    }
}