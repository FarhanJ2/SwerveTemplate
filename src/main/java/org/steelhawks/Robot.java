// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.steelhawks;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot
{
    private Command autonomousCommand;

    public enum RobotState {
        DISABLED,
        TELEOP,
        AUTON,
        TEST
    }

    public static RobotState state = RobotState.DISABLED;
    
    public Robot() {
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin, 0, WPILibVersion.Version);
        DriverStation.silenceJoystickConnectionWarning(true);

        // Initialize Robot Container
        new RobotContainer();
    }

    int counter = 0;
    @Override
    public void robotPeriodic()
    {
        counter = (counter + 1) % 1000;

        CommandScheduler.getInstance().run();

        /* Update SmartDashboard every 5 cycles (10 time a second) */
        if (counter % 5 == 0) {
            SmartDashboard.putString("robot/state", state.toString());
            SmartDashboard.putString("auton/auton selected", RobotContainer.s_Autos.getAutonName());
        }
    }
    
    
    @Override
    public void disabledInit() {
        state = RobotState.DISABLED;

        RobotContainer.s_LED.getRainbowCommand();
    }
    
    
    @Override
    public void disabledPeriodic() {
        RobotContainer.s_LED.rainbow();
    }
    
    
    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit()
    {
        autonomousCommand = RobotContainer.s_Autos.getAutonomousCommand();
        
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }

        state = RobotState.AUTON;
    }
    
    
    @Override
    public void autonomousPeriodic() {}
    
    
    @Override
    public void autonomousExit() {}
    
    
    @Override
    public void teleopInit()
    {
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }

        state = RobotState.TELEOP;
    }
    
    
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void teleopExit() {}
    
    
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
        state = RobotState.TEST;
    }
    
    
    @Override
    public void testPeriodic() {}
    
    
    @Override
    public void testExit() {}
}
