package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    @AutoLog
    class GyroIOInputs {
        public boolean isConnected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawPositionRadPerSecond = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}
    default Rotation2d getGyroYaw() {return new Rotation2d();}
}
