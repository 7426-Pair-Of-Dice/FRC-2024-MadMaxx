// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.body.BodyConstants.Setpoint;
import frc.robot.subsystems.body.Arm;
import frc.robot.subsystems.body.Elevator;

public class UpdateSetpoint extends Command {
  private Arm m_arm;
  private Elevator m_elevator;
  private Setpoint m_setpoint;

  protected Timer m_timer = new Timer();
  private final double m_armDelay;
  private final double m_elevatorDelay;

  public UpdateSetpoint(Arm arm, Elevator elevator, Setpoint setpoint) {
    this(arm, elevator, setpoint, 0.0, 0.0);
  }

  public UpdateSetpoint(Arm arm, Elevator elevator, Setpoint setpoint, double armDelay, double elevatorDelay) {
    m_arm = arm;
    m_elevator = elevator;
    m_setpoint = setpoint;

    m_armDelay = armDelay;
    m_elevatorDelay = elevatorDelay;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_armDelay <= 0.0 || m_timer.hasElapsed(m_armDelay)) {
      m_arm.updateSetpoint(m_setpoint);
    }

    if(m_elevatorDelay <= 0.0 || m_timer.hasElapsed(m_elevatorDelay)) {
      m_elevator.updateSetpoint(m_setpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (
      (m_armDelay <= 0.0 || m_timer.hasElapsed(m_armDelay))
      && (m_elevatorDelay <= 0.0 || m_timer.hasElapsed(m_elevatorDelay))
    );
  }
}
