package org.steelhawks.subsystems.elevator;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RealElevator implements ElevatorIO {

    private final TalonFX m_elevatorMotor = new TalonFX(15, "canivore");
    private final CANcoder m_elevatorEncoder = new CANcoder(14, "canivore");

    private final ElevatorFeedforward m_elevatorFeedForward = new ElevatorFeedforward(0, 0, 0);
    private final ProfiledPIDController m_elevatorController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    private TrapezoidProfile.State m_setpoint;
    private double m_lastVoltage;

    @Override
    public double getHeight() {
        return m_elevatorEncoder.getPosition().getValueAsDouble();
    }

    @Override
    public TrapezoidProfile.State getCurrentState() {
        return m_elevatorController.getSetpoint();
    }

    @Override
    public TrapezoidProfile.State getDesiredState() {
        return m_setpoint;
    }

    @Override
    public void setDesiredState(double setpoint) {
        m_elevatorController.setGoal(setpoint);
    }

    @Override
    public void updateSetpoint(TrapezoidProfile.State setpoint) {
        double ff = m_elevatorFeedForward.calculate(setpoint.position, setpoint.velocity);
        m_lastVoltage = m_elevatorController.calculate(getHeight(), setpoint.position) + ff;

        m_elevatorMotor.setVoltage(m_lastVoltage);
        m_setpoint = setpoint;
    }
}
