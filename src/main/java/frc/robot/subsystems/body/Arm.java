// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.shared.Alert;
import frc.robot.shared.Constants;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.BodyConstants.Setpoint;
import frc.robot.shared.Alert.AlertType;
import static frc.robot.subsystems.body.BodyConstants.*;

public class Arm extends SubsystemBase {
  private static Arm m_instance;
  
  public static Arm getInstance() {
    if(m_instance == null) m_instance = new Arm();
    return m_instance;
  }

  private static TalonFX m_topMotor;
  private static TalonFX m_bottomMotor;

  private static MotionMagicVoltage m_positionOut;
  private static DutyCycleOut m_percentOut;

  private static CANcoder m_encoder; 

  private static Setpoint m_setpoint;

  private static ArmState m_state;

  private static double m_customAngle = 32.0;
  public double m_temporaryRemove = 0.02;
  private static double m_closedLoopPosition = 20.0;

  private Alert m_topTempWarning;
  private Alert m_topTempDanger;
  private Alert m_bottomTempWarning;
  private Alert m_bottomTempDanger;

  private static double m_threshold = 0.0;

  public static enum ArmState {
    Setpoint,
    ClosedLoop,
    Debug,
    Manual;
  }
  
  private Arm() {
    m_topMotor = new TalonFX(kArmConfig.topId());
    m_bottomMotor = new TalonFX(kArmConfig.bottomId());
    m_encoder = new CANcoder(kArmConfig.encoderId());

    configMotor(m_topMotor.getConfigurator());
    configMotor(m_bottomMotor.getConfigurator());

    // m_topMotor.optimizeBusUtilization();
    // m_bottomMotor.optimizeBusUtilization();

    m_bottomMotor.setControl(new Follower(m_topMotor.getDeviceID(), false));

    m_positionOut = new MotionMagicVoltage(0.0);
    m_positionOut.Slot = 0;
    m_percentOut = new DutyCycleOut(0.0);

    m_setpoint = Setpoint.Idle;
    m_state = ArmState.Setpoint;
    
    m_topTempWarning = new Alert("Top arm motor has exceeded 35 degrees celcius", AlertType.WARNING);
    m_topTempDanger = new Alert("Top arm motor has exceeded 45 degrees celcius, cool down the motor if possible!", AlertType.ERROR);
    m_bottomTempWarning = new Alert("Bottom arm motor has exceeded 35 degrees celcius", AlertType.WARNING);
    m_bottomTempDanger = new Alert("Bottom arm motor has exceeded 45 degrees celcius, cool down the motor if possible!", AlertType.ERROR);
  }
  
  private static void configMotor(TalonFXConfigurator config) {
    var newConfig = new TalonFXConfiguration();

    // Set feedback sensor to CANCoder
    FeedbackConfigs feedback = newConfig.Feedback;
    feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    
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

    // Configure PID in Slot 1
    Slot1Configs slot1 = newConfig.Slot1;
    slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot1.kP = 0.0;
    slot1.kI = 0.0;
    slot1.kD = 0.0;
    slot1.kS = 0.5;
    slot1.kV = 0.12;

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicAcceleration = 1500.0;
    motionMagic.MotionMagicCruiseVelocity = 4000.0;
    motionMagic.MotionMagicJerk = 6000.0;

    //Configing the arm encoder
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // encoderConfig.MagnetSensor.MagnetOffset = 0.0029296875;
    m_encoder.getConfigurator().apply(encoderConfig, 0.05);

    config.apply(newConfig, 0.050);
  }

  private void checkTemperature() {
    double topTemp = m_topMotor.getDeviceTemp().getValueAsDouble();
    double bottomTemp = m_bottomMotor.getDeviceTemp().getValueAsDouble();
    m_topTempWarning.set(topTemp > 35 && topTemp < 45);
    m_bottomTempWarning.set(bottomTemp > 35 && bottomTemp < 45);

    m_topTempDanger.set(topTemp > 45);
    m_bottomTempDanger.set(bottomTemp > 45);
  }

  @Override
  public void periodic() {
    checkTemperature();
    switch(m_state) {
      case Setpoint:
        moveToSetpoint();
        break;
      case ClosedLoop:
        moveToPosition();
        break;
      case Debug:
        moveToDebug();
        break;
      default:
        setMotors(0.0);
        break;
    }
  }

  public double getRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble() * 360;
  }
  
  public double getDegrees() {
    return getRotations();
  }


  public double getError() {
    if(m_state == ArmState.Setpoint) {
      return getDegrees() - m_setpoint.getDegrees();
    } else return getDegrees() - m_customAngle;
  }


  public double getSetpoint() {
    return m_setpoint.getDegrees();
  }

  public double getElevatorFeed() {
    double angle = getDegrees();
    return (-0.3 + (-0.08 * angle) + (4.26E-03 * Math.pow(angle,  2)) + (-3.63E-05 * Math.pow(angle,  3)));
  }

  public void setPoint(Setpoint setpoint) {
    m_setpoint = setpoint;    
  }

  public boolean forwardLimitStatus() {
    double currentPosition = getRotations();
    return kArmLimits.forwardLimit() < currentPosition;
  }

  public boolean reverseLimitStatus() {
    double currentPosition = getRotations();
    return kArmLimits.reverseLimit() > currentPosition;
  }

  public boolean setpointReached() {
    return Math.abs(getError()) < m_threshold;
  }

  public void moveToSetpoint() {
    m_topMotor.setControl(
      m_positionOut
        .withPosition(m_setpoint.getDegrees() / 360)
        .withSlot(0)
        .withFeedForward(m_setpoint.getArmFeed())
    );
  }

  public void moveToPosition() {
    m_topMotor.setControl(
      m_positionOut
        .withPosition(Limelight.calculation.angle() / 360)
        .withSlot(0)
        .withFeedForward(m_setpoint.getArmFeed())
    );
  }

  public void moveToDebug() {
    m_topMotor.setControl(
      m_positionOut
        .withPosition(m_customAngle / 360)
        .withSlot(0)
    );
  }

  public void setPosition(double angle) {
    m_closedLoopPosition = angle;
  }


  public void setMotors(double percent) {
    m_percentOut.Output = percent;
    m_topMotor.setControl(m_percentOut);
  }

  public void setState(ArmState state) {
    m_state = state;
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }

  public void stop() {
    m_percentOut.Output = 0.0;
    m_topMotor.setControl(new StaticBrake());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
    
    if(Constants.Dashboard.kSendErrors) {
      builder.addDoubleProperty("Error", this::getError, null);
    }
    
    if(Constants.Dashboard.kSendReferences) {
      builder.addDoubleProperty("Reference", this::getSetpoint, null);
    }

    if(Constants.Dashboard.kSendStates) {
      builder.addDoubleProperty("Position", this::getDegrees, null);
    }

    if(Constants.Dashboard.kSendDebug) {
      builder.addDoubleProperty("Feed Forward", this::getElevatorFeed, null);
      builder.addStringProperty("Setpoint Name", ()->{return m_setpoint.name();}, null);

      builder.addDoubleProperty("Top Arm Stator", ()->{return m_topMotor.getStatorCurrent().getValueAsDouble();}, null);
      builder.addDoubleProperty("Bottom Arm Stator", ()->{return m_bottomMotor.getStatorCurrent().getValueAsDouble();}, null);

      builder.addDoubleProperty("Debug Angle", ()->{return m_customAngle;}, (double value)->{m_customAngle=value;});
      builder.addDoubleProperty("Limelight kP", ()->{return m_temporaryRemove;}, (double value)->{m_temporaryRemove = value;});
    }
  }
}
