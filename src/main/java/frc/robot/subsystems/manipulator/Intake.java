// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.Constants;
import frc.robot.subsystems.leds.Leds;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

public class Intake extends SubsystemBase {
  private static Intake m_instance;
  
  public static Intake getInstance() {
    if(m_instance == null) m_instance = new Intake();
    return m_instance;
  }

  private static CANSparkMax m_topMotor;
  private static CANSparkMax m_bottomMotor;

  private static DigitalInput m_lowBrake;
  private static DigitalInput m_highBrake;

  private static final int kLowBrakeId = 1;
  private static final int kHighBrakeId = 2;

  private Intake() {
    m_topMotor = new CANSparkMax(kIntakeConfig.topId(), MotorType.kBrushless);
    m_bottomMotor = new CANSparkMax(kIntakeConfig.bottomId(), MotorType.kBrushless);
    
    m_lowBrake = new DigitalInput(kLowBrakeId);
    m_highBrake = new DigitalInput(kHighBrakeId);

    configMotor(m_topMotor);
    configMotor(m_bottomMotor, m_topMotor);
  }

  private static void configMotor(CANSparkMax motor) {
    configMotor(motor, null);
  }

  private static void configMotor(CANSparkMax motor, CANSparkMax leader) {
    // Restoring factory settings to ensure that we get the same results every time
    motor.restoreFactoryDefaults();
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(true);
    if(motor != null) motor.follow(leader);
    motor.setSmartCurrentLimit((int)kIntakeLimits.supplyLimit());
    motor.clearFaults();
    motor.burnFlash();
  }

  @Override
  public void periodic() {
    // Set flag on our Leds instance if a note is detected or not.
    Leds.getInstance().NoteDetected = highBrake();
  }

  /**
   * Sets the percent output on the top and bottom intake roller.
   * @param percent The percent to set. Value should be between -1.0 and 1.0
   */
  public void setPercent(double percent) {
    m_topMotor.set(percent);
  }

  /**
   * Stops both intake rollers.
   */
  public void stop() {
    m_topMotor.set(0.0);
  }

  /**
   * @return The current state of the top beam brake.
   */
  public boolean highBrake() {
    return !m_highBrake.get();
  }

  /**
   * @return The current state of the bottom beam brake.
   */
  public boolean lowBrake() {
    return !m_lowBrake.get();
  }

  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");
    
    if(Constants.Dashboard.kSendStates) {
      builder.addBooleanProperty("High Brake", this::highBrake, null);
      builder.addBooleanProperty("Low Brake", this::lowBrake, null);
    }

  }
}
