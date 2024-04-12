// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Shooter;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.Arm;
import frc.robot.subsystems.body.Arm.ArmState;

public class AutoAim extends Command {
  private Arm m_arm = Arm.getInstance();
  private Shooter m_shooter = Shooter.getInstance();
  private boolean m_stopOnEnd = false;

  public AutoAim() {
    this(true);
  }

  public AutoAim(boolean continueOnEnd) {
    addRequirements(m_arm, m_shooter);
    m_stopOnEnd = continueOnEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setState(ArmState.ClosedLoop);
    System.out.println("[SETPOINT] Auto Aim Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_arm.setPosition(Limelight.calculation.angle());
      m_shooter.setMotors(Limelight.calculation.rps());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_stopOnEnd) {
      m_arm.setState(ArmState.Setpoint);
      m_shooter.stop();
      System.out.println("[SETPOINT] Auto Aim Stop");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
