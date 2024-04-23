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

  // Get subystem insances
  private Arm m_arm = Arm.getInstance();
  private Shooter m_shooter = Shooter.getInstance();

  /** 
   * This boolean defaults to true, and if true will return the arm
   * to following setpoints and stop the shooter. False is intended to be used
   * within autonomous.
   */
  private boolean m_stopOnEnd;

  /**
   * This command utilizes calculations from the Limelight to automatically
   * set the Arm's position and the Shooter's RPS in order to attempt a shot
   * from anywhere within the wing.
   */
  public AutoAim() {
    this(true);
  }

  /**
   * This command utilizes calculations from the Limelight to automatically
   * set the Arm's position and the Shooter's RPS in order to attempt a shot
   * from anywhere within the wing.
   * 
   * @param continueOnEnd Determines whether the arm and shooter return to normal functions once the command is interrupted.
   */
  public AutoAim(boolean continueOnEnd) {
    m_stopOnEnd = continueOnEnd;
    addRequirements(m_arm, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set Arm control mode to Closed Loop
    m_arm.setState(ArmState.ClosedLoop);

    System.out.println("[SETPOINT] Auto Aim Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // Set the arm position and shooter RPS to the Limelight's calculation.
      m_arm.setPosition(Limelight.calculation.angle());
      m_shooter.setVelocity(Limelight.calculation.rps());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_stopOnEnd) {
      // Set Arm control mode to Setpoint
      m_arm.setState(ArmState.Setpoint);
      m_shooter.stop(); // Stop shooter

      System.out.println("[SETPOINT] Auto Aim Stop");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
