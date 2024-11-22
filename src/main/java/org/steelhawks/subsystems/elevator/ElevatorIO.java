package org.steelhawks.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface ElevatorIO {
    /** Returns the height of the elevator in rotations */
    double getHeight();
    /** Returns the current state of the elevator (rad/s) */
    TrapezoidProfile.State getCurrentState();
    /** Returns the desired state of the elevator (rad/s) */
    TrapezoidProfile.State getDesiredState();
    /** Sets the desired state of the elevator (rad/s) */
    void setDesiredState(double setpoint);
    /** Updates the setpoint of the elevator */
    void updateSetpoint(TrapezoidProfile.State setpoint);
}
