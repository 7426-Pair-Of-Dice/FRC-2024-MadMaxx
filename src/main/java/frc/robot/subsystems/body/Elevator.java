// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.Constants;
import frc.robot.subsystems.body.BodyConstants.Setpoint;

import static frc.robot.subsystems.body.BodyConstants.*;

public class Elevator extends SubsystemBase {
  private static Elevator m_instance;
  
  public static Elevator getInstance() {
    if(m_instance == null) m_instance = new Elevator();
    return m_instance;
  }

  private final TalonFX m_leftMotor;
  private final TalonFX m_rightMotor;
  private final CANcoder m_encoder;

  private final MotionMagicVoltage m_positionOut;

  private Setpoint m_setpoint;

  private Elevator() {
    // Creating new motors and encoder
    m_leftMotor = new TalonFX(kElevatorConfig.leftId());
    m_rightMotor = new TalonFX(kElevatorConfig.rightId());
    m_encoder = new CANcoder(kElevatorConfig.encoderId());

    // Creating new control modes
    m_positionOut = new MotionMagicVoltage(0.0).withSlot(0);

    // Setting default setpoint
    m_setpoint = Setpoint.Idle;

    // Configuring motors
    configMotor(m_leftMotor.getConfigurator(), m_encoder);
    configMotor(m_rightMotor.getConfigurator());

    // Set right motor to follow the left
    m_rightMotor.setControl(new Follower(m_leftMotor.getDeviceID(), false));
  }
  
  private static void configMotor(TalonFXConfigurator config) {
    configMotor(config, null);
  }

  private static void configMotor(TalonFXConfigurator config, CANcoder encoder) {
    var newConfig = new TalonFXConfiguration();

    if(encoder != null) {
      // Set feedback sensor to CANCoder
      FeedbackConfigs feedback = newConfig.Feedback;
      feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
      feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

      //Configing the elevator encoder
      var encoderConfig = new CANcoderConfiguration();
      encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
      encoderConfig.MagnetSensor.MagnetOffset = 0;
      encoder.getConfigurator().apply(encoderConfig, 0.05);
      encoder.setPosition(0.0);

      encoder.getConfigurator().apply(encoderConfig, 0.05);
    }

    // Set soft limits
    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = kElevatorLimits.forwardLimit();
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = kElevatorLimits.reverseLimit();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.CounterClockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 12;
    voltage.PeakReverseVoltage = -8;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kElevatorLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kElevatorLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0
    // 0 to 30 degrees
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = 2.0;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kS = 0.25;
    slot0.kV = 0.0;
    
    // Configure PID in Slot 1
    // 30 to 75 degrees
    Slot1Configs slot1 = newConfig.Slot1;
    slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot1.kP = 2.0;
    slot1.kI = 0.0;
    slot1.kD = 0.0;
    slot1.kS = 0.3;
    slot1.kV = 0.0;
    
    // Configure PID in Slot 2
    // 75+ degrees 
    Slot2Configs slot2 = newConfig.Slot2;
    slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot2.kP = 2.5;
    slot2.kI = 0.0;
    slot2.kD = 0.0;
    slot2.kS = 0.5;
    slot2.kV = 0.0;
    
    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicAcceleration = 160.0;
    motionMagic.MotionMagicCruiseVelocity = 320.0;
    motionMagic.MotionMagicJerk = 1600.0;
    
    config.apply(newConfig, 0.050);
  }

  @Override
  public void periodic() {
    moveToSetpoint();
  }

  /**
   * Moves the elevator motors toward the current reference.
   */
  public void moveToSetpoint() {
    m_leftMotor.setControl(
      m_positionOut
      .withPosition(m_setpoint.getRotations())
      .withSlot(m_setpoint.getElevatorSlot())
      .withFeedForward(m_setpoint.getElevatorFeed()));
  }

  /**
   * Updates the current setpoint for the elevator to reference.
   * @param setpoint A pre-determined setpoint.
   */
  public void updateSetpoint(Setpoint setpoint) {
    m_setpoint = setpoint;
  }

  /**
   * @return Rotations of the absolute CANCoder on the elevator.
   */
  public double getRotations() {
    return m_encoder.getPosition().getValueAsDouble(); 
  }

  /**
   * @return The desired elevator position in rotations.
   */
  public double getReference() {
    return m_setpoint.getRotations();
  }

  /**
   * @return The error between the actual position versus the desired position, in rotations.
   */
  public double getError() {
    return getRotations() - getReference();
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Elevator");
    
    if(Constants.Dashboard.kSendErrors) {
      builder.addDoubleProperty("Error", this::getError, null);
    }
    
    if(Constants.Dashboard.kSendReferences) {
      builder.addDoubleProperty("Reference", this::getReference, null);
    }

    if(Constants.Dashboard.kSendStates) {
    builder.addDoubleProperty("Position", this::getRotations, null);
    }

    if(Constants.Dashboard.kSendDebug) {
      builder.addStringProperty("Setpoint Name", ()->{return m_setpoint.name();}, null);
    }

  }
}
