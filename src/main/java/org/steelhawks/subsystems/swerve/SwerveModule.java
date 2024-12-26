package org.steelhawks.subsystems.swerve;


import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

    public int moduleNumber;
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public SwerveModule(int moduleNumber, ModuleIO io) {
        this.io = io;
        this.moduleNumber = moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        io.setDesiredState(desiredState, isOpenLoop);
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        io.setSpeed(desiredState, isOpenLoop);
    }

    public void resetToAbsolute() {
        io.resetToAbsolute();
    }

    public SwerveModuleState getState() {
        return io.getState();
    }

    public SwerveModulePosition getPosition() {
        return io.getPosition();
    }

    public double getDriveVoltage() {
        return io.getDriveVoltage();
    }

    public void runSysId(double driveVoltage) {
        io.setRawVoltage(driveVoltage);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Swerve/Mod" + moduleNumber, inputs);
    }
}