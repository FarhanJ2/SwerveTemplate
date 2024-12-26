package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;
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
    private final VelocityVoltage driveVelocityVoltage = new VelocityVoltage(0);

    /** Used for SYSID Characterization */
    private final VoltageOut driveVoltageOut = new VoltageOut(0);

    /* angle motor control requests */
    private final PositionVoltage anglePositionVoltage = new PositionVoltage(0);

    /* StatusSignals */
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;

    private final StatusSignal<Double> anglePosition;
    private final StatusSignal<Double> angleVelocity;

    private boolean isOpenLoop = false;

    public RealModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.getAngleOffset();

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.getCancoderID(), Constants.CANIVORE_NAME);
        angleEncoder.getConfigurator().apply(KSwerve.CONFIGS.swerveCANcoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.getAngleMotorID(), Constants.CANIVORE_NAME);
        mAngleMotor.getConfigurator().apply(KSwerve.CONFIGS.swerveAngleFXConfig);

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.getDriveMotorID(), Constants.CANIVORE_NAME);
        mDriveMotor.getConfigurator().apply(KSwerve.CONFIGS.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        drivePosition = mDriveMotor.getPosition();
        driveVelocity = mDriveMotor.getVelocity();


        anglePosition = mAngleMotor.getPosition();
        angleVelocity = mAngleMotor.getVelocity();


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
        this.isOpenLoop = isOpenLoop;
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        mAngleMotor.setControl(anglePositionVoltage.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    @Override
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / KSwerve.MAX_SPEED;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocityVoltage.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, KSwerve.WHEEL_CIRCUMFERENCE);
            driveVelocityVoltage.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocityVoltage);
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
    public void setRawVoltage(double driveVoltage) {
        driveVoltageOut.Output = driveVoltage;
        mDriveMotor.setControl(driveVoltageOut);
    }

    @Override
    public double getDriveVoltage() {
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

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveAppliedVoltage = getDriveVoltage();
        inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.anglePosition = Rotation2d.fromRadians(Units.rotationsToRadians(anglePosition.getValueAsDouble()));
        inputs.angleVelocityRadsPerSec = Units.rotationsToRadians(angleVelocity.getValueAsDouble());
        inputs.desiredState = getState();
        inputs.isOpenLoop = isOpenLoop;
    }
}