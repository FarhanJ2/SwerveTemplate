package org.steelhawks.subsystems.swerve;

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
import org.steelhawks.Constants;
import org.steelhawks.RobotContainer;
import org.steelhawks.lib.Conversions;
import org.steelhawks.lib.SwerveModuleConstants;

public class RealModule implements ModuleIO {

    public int moduleNumber;
    private final Rotation2d angleOffset;

    private final TalonFX mAngleMotor;
    private final TalonFX mDriveMotor;
    private final CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(KSwerve.driveKS, KSwerve.driveKV, KSwerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public RealModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
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

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / KSwerve.MAX_SPEED;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, KSwerve.WHEEL_CIRCUMFERENCE);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        mAngleMotor.setPosition(absolutePosition);
    }

    @Override
    public double getVoltage() {
        return mDriveMotor.getMotorVoltage().getValueAsDouble() + mAngleMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), KSwerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), KSwerve.WHEEL_CIRCUMFERENCE),
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }
}