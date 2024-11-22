package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimElevator implements ElevatorIO {

    private final DCMotorSim mLeftElevatorMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 1);
    private final DCMotorSim mRightElevatorMotor = new DCMotorSim(DCMotor.getFalcon500(1), 1, 1);

    private TrapezoidProfile.State mSetpoint;

    @Override
    public double getHeight() {
        return mLeftElevatorMotor.getAngularPositionRotations();
    }

    @Override
    public TrapezoidProfile.State getCurrentState() {
        return null;
    }

    @Override
    public TrapezoidProfile.State getDesiredState() {
        return null;
    }

    @Override
    public void setDesiredState(double setpoint) {

    }

    @Override
    public void updateSetpoint(TrapezoidProfile.State setpoint) {


//        mLeftElevatorMotor.setInputVoltage();
//        mRightElevatorMotor.setInputVoltage();

        mSetpoint = setpoint;
    }
}
