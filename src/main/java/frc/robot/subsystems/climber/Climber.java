// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static Climber m_instance;
  
  public static Climber getInstance() {
    if(m_instance == null) m_instance = new Climber();
    return m_instance;
  }

  private final TalonFX m_motor;

  private final DutyCycleOut m_output;

  private static final int kMotorId = 25;

  private Climber() {
    m_motor = new TalonFX(kMotorId);

    m_output = new DutyCycleOut(0.0);

    configMotor(m_motor.getConfigurator());

    m_motor.optimizeBusUtilization();
  }

  private static void configMotor(TalonFXConfigurator config) {
    // Creating a new configuration to ensure we get the same results every time
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;
    
    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 6;
    voltage.PeakReverseVoltage = -6;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  /**
   * Sets the percent output on the climb motor.
   * @param percent Proportion of supply voltage to apply in fractional units between -1 and +1
   */
  public void setPercent(double percent) {
    m_motor.setControl(m_output.withOutput(percent));
  }

  /**
   * Stops climb motor.
   */
  public void stop() {
    m_motor.setControl(m_output.withOutput(0.0));
  }
}
