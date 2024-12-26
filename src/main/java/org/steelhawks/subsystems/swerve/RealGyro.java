package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

public class RealGyro implements GyroIO {

    private final StatusSignal<Double> yawVelocity;
    private final Pigeon2 mGyro;

    public RealGyro() {
        mGyro = new Pigeon2(KSwerve.PIGEON_ID, Constants.CANIVORE_NAME);
        mGyro.getConfigurator().apply(new Pigeon2Configuration());
        mGyro.setYaw(0);

        yawVelocity = mGyro.getAngularVelocityZWorld();
        yawVelocity.setUpdateFrequency(100.0);
        mGyro.optimizeBusUtilization();
    }

    @Override
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw().getValue());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(mGyro.getYaw(), yawVelocity).isOK();
        inputs.yawPosition = Rotation2d.fromDegrees(mGyro.getYaw().getValueAsDouble());
        inputs.yawPositionRadPerSecond = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}
