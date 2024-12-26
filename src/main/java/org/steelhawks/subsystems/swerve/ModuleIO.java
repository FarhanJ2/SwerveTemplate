package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    class ModuleIOInputs {
        public double drivePositionRads;
        public double driveVelocityRadsPerSec;
        public Rotation2d anglePosition;
        public double angleVelocityRadsPerSec;
        public SwerveModuleState desiredState;
        public boolean isOpenLoop;
        public double driveAppliedVoltage;
    }

    default void updateInputs(ModuleIOInputs inputs) {}
    default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}
    default void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {}
    default void resetToAbsolute() {}
    default void setRawVoltage(double driveVoltage) {}
    default SwerveModuleState getState() { return new SwerveModuleState(); }
    default SwerveModulePosition getPosition() { return new SwerveModulePosition(); }
    /** Returns 0 volts if it is a simulation, returns drive and angle motor voltage usage if real. */
    default double getDriveVoltage() { return 0; }
}
