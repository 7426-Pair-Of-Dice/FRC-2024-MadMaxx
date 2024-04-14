// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.RobotState;
import frc.robot.lib.Controls.OI;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.leds.Leds;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double m_autoStart = 0.0;
  private double m_teleStart = 0.0;

  private boolean m_autoTimeLogged = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    RobotController.setBrownoutVoltage(6.5);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    OI.getInstance().checkConnections();
    Leds.getInstance().periodic();
    Limelight.periodic();

    if(m_autonomousCommand != null){
      if (m_autonomousCommand != null && !m_autonomousCommand.isScheduled() && !m_autoTimeLogged) {
          if (DriverStation.isAutonomousEnabled()) 
            System.out.printf("[AUTONOMOUS] Command completed in %.2f seconds.%n", Timer.getFPGATimestamp() - m_autoStart);
          else 
            System.out.printf("[AUTONOMOUS] Command completed in %.2f seconds.%n", Timer.getFPGATimestamp() - m_autoStart);
          
        m_autoTimeLogged = true;
        Leds.getInstance().AutonomousFinished = true;
        Leds.getInstance().AutonomousEnd = Timer.getFPGATimestamp();
      } 
    }
  }

  @Override
  public void disabledInit() {
    
    m_robotContainer.setRobotState(RobotState.Disabled);
  }

  @Override
  public void disabledPeriodic() {
    

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autoStart = Timer.getFPGATimestamp();
    m_robotContainer.setRobotState(RobotState.Autonomous);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_teleStart = Timer.getFPGATimestamp();
    m_robotContainer.setRobotState(RobotState.Teleop);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    m_robotContainer.setRobotState(RobotState.Test);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
