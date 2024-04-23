// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

public class Arm extends SubsystemBase {
  private static Arm m_instance;
  
  public static Arm getInstance() {
    if(m_instance == null) m_instance = new Arm();
    return m_instance;
  }

  private final TalonFX m_topMotor;
  private final TalonFX m_bottomMotor;
  private final CANcoder m_encoder;

  private final MotionMagicVoltage m_positionOut;

  private ArmState m_state;
  private Setpoint m_setpoint;

  private static double m_angle = 20.0;

  private static double m_reference = 0.0;

  public static enum ArmState {
    Setpoint,
    ClosedLoop;
  }
  
  private Arm() {
    // Creating new motors and encoder
    m_topMotor = new TalonFX(kArmConfig.topId());
    m_bottomMotor = new TalonFX(kArmConfig.bottomId());
    m_encoder = new CANcoder(kArmConfig.encoderId());

    // Creating new control modes
    m_positionOut = new MotionMagicVoltage(0.0).withSlot(0);

    // Setting default state and setpoint
    m_state = ArmState.Setpoint;
    m_setpoint = Setpoint.Idle;

    // Configuring motors
    configMotor(m_topMotor.getConfigurator(), m_encoder);
    configMotor(m_bottomMotor.getConfigurator());

    // Set bottom motor to follow the top
    m_bottomMotor.setControl(new Follower(m_topMotor.getDeviceID(), false));
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

      //Configing the arm encoder
      var encoderConfig = new CANcoderConfiguration();
      encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
      encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

      encoder.getConfigurator().apply(encoderConfig, 0.05);
    }
    
    // Set soft limits
    var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = kArmLimits.forwardLimit();
    limits.ReverseSoftLimitEnable = true;
    limits.ReverseSoftLimitThreshold = kArmLimits.reverseLimit();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.CounterClockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 16;
    voltage.PeakReverseVoltage = -16;

    // Set ramp period (0.02 - 0.05 secs)
    var ramp = newConfig.ClosedLoopRamps;
    ramp.VoltageClosedLoopRampPeriod = 0.05;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kArmLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kArmLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = 175.0;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kS = 0.2;
    slot0.kG = 0.5;
    slot0.kV = 0.0;

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicAcceleration = 1500.0;
    motionMagic.MotionMagicCruiseVelocity = 4000.0;
    motionMagic.MotionMagicJerk = 6000.0;

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  @Override
  public void periodic() {
    updateReference(
      m_state == ArmState.Setpoint ? m_setpoint.getDegrees() : m_angle
    );
  }

  /**
   * Updates the desired reference angle for the arm.
   * @param angle A position to drive towards in degrees.
   */
  public void updateReference(double angle) {
    m_reference = angle;
    m_topMotor.setControl(
      m_positionOut
        .withPosition(angle / 360)
        .withSlot(0)
        .withFeedForward(m_setpoint.getArmFeed())
    );
  }

  /**
   * Updates the current setpoint for the arm to reference
   * in the arm state "Setpoint".
   * @param setpoint A pre-determined setpoint.
   */
  public void updateSetpoint(Setpoint setpoint) {
    m_setpoint = setpoint;    
  }

  /**
   * Updates the current angle for the arm to reference
   * in the arm state "ClosedLoop".
   * @param angle The desired arm angle in degrees.
   */
  public void setPosition(double angle) {
    m_angle = angle;
  }

  /**
   * Updates the arm state.
   * @param state Setpoint or Closed-Loop
   */
  public void setState(ArmState state) {
    m_state = state;
  }

  /**
   * @return Rotations of the absolute CANCoder on the arm.
   */
  public double getRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }
  
  /**
   * @return Degrees calculated from the current CANCoder rotations.
   */
  public double getDegrees() {
    return getRotations() * 360;
  }

  /**
   * @return The desired arm position in degrees.
   */
  public double getReference() {
    return m_reference;
  }
  
  /**
   * @return The error between the actual position versus the desired position, in degrees.
   */
  public double getError() {
    return getDegrees() - m_reference;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
    
    if(Constants.Dashboard.kSendErrors) {
      builder.addDoubleProperty("Error", this::getError, null);
    }
    
    if(Constants.Dashboard.kSendReferences) {
      builder.addDoubleProperty("Reference", this::getReference, null);
    }

    if(Constants.Dashboard.kSendStates) {
      builder.addDoubleProperty("Position", this::getDegrees, null);
    }

    if(Constants.Dashboard.kSendDebug) {
      builder.addStringProperty("Setpoint Name", ()->{return m_setpoint.name();}, null);
    }
  }
}
