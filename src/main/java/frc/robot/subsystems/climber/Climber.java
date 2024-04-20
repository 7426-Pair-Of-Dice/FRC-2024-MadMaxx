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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.Constants;


public class Climber extends SubsystemBase {
  private static Climber m_instance;
  
  public static Climber getInstance() {
    if(m_instance == null) m_instance = new Climber();
    return m_instance;
  }

  private final int m_motorId = 25;

  private final TalonFX m_motorOne = new TalonFX(m_motorId);

  // This seems to be what you would use to determine the power sent to the motor
  private final DutyCycleOut m_outputOne = new DutyCycleOut(0.0);


  /** Creates a new Climber. */
  private Climber() {
    configMotor(m_motorOne.getConfigurator());

    m_motorOne.optimizeBusUtilization();
  }

  private static void configMotor(TalonFXConfigurator config) {
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;
    
    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 6;
    voltage.PeakReverseVoltage = -6;

    config.apply(newConfig, 0.050);
  }

  public double getRotations() {
    return m_motorOne.getPosition().getValueAsDouble();
  }

  public void zeroEncoder() {
    m_motorOne.setPosition(0);
  }

  public void setMotor(double percent) {
    m_motorOne.setControl(m_outputOne.withOutput(percent));
  }

  public void stop() {
    m_motorOne.setControl(m_outputOne.withOutput(0.0));
  }

  @Override
  public void periodic() {}
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");

    if(Constants.Dashboard.kSendStates) {
      builder.addDoubleProperty("Position", this::getRotations, null);
    }
  }
}
