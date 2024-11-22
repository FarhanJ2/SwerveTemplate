package org.steelhawks.subsystems.elevator;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.steelhawks.Robot;

public class Elevator extends SubsystemBase {

    private final ElevatorIO mElevatorIO;

    private static Elevator INSTANCE;

    public static Elevator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Elevator();
        }
        return INSTANCE;
    }

    private Elevator() {

        if (Robot.isReal()) {
            mElevatorIO = new RealElevator();
        } else {
            mElevatorIO = new SimElevator();
        }
    }

    @Override
    public void periodic() {
        mElevatorIO.updateSetpoint(mElevatorIO.getDesiredState());
    }

    ///////////////////////
    /* COMMAND FACTORIES */
    ///////////////////////


}

