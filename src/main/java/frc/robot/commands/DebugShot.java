// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Shooter;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Arm;
import frc.robot.subsystems.body.Arm.ArmState;

public class DebugShot extends Command {
  private Arm m_arm = Arm.getInstance();
  private Shooter m_shooter = Shooter.getInstance();

  public DebugShot() {
    addRequirements(m_arm, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setState(ArmState.Debug);
    System.out.println("[SETPOINT] Auto Aim Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_shooter.setMotors(70);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setState(ArmState.Setpoint);
    m_shooter.stop();
    System.out.println("[SETPOINT] Auto Aim Stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
