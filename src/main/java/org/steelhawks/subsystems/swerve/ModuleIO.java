package org.steelhawks.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {
    void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop);
    void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop);
    void resetToAbsolute();
    SwerveModuleState getState();
    SwerveModulePosition getPosition();
    /** Returns 0 volts if it is a simulation, returns drive and angle motor voltage usage if real. */
    double getVoltage();

    /** For logging on AdvantageKit */
    class ModuleIOInputs implements LoggableInputs {
        public SwerveModuleState desiredState;
        public boolean isOpenLoop;


        @Override
        public void toLog(LogTable table) {
            table.put("desiredState", desiredState);
            table.put("isOpenLoop", isOpenLoop);
        }

        @Override
        public void fromLog(LogTable table) {
            desiredState = table.get("desiredState", desiredState);
            isOpenLoop = table.get("isOpenLoop", isOpenLoop);
        }
    }

    default void updateInputs(ModuleIOInputs inputs) {}
}
