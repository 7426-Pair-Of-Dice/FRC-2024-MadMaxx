// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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

  private static RelativeEncoder m_encoder;
  private static SparkPIDController m_pidController;

  private static DigitalInput m_lowBrake;
  private static DigitalInput m_highBrake;
  private static int kLowBrakeId = 1;
  private static int kHighBrakeId = 2;

  private static boolean m_lastBeamState = false;
  private static double m_noteSeenPosition = 0.0;

  private Intake() {
    m_topMotor = new CANSparkMax(kIntakeConfig.topId(), MotorType.kBrushless);
    m_bottomMotor = new CANSparkMax(kIntakeConfig.bottomId(), MotorType.kBrushless);
    m_lowBrake = new DigitalInput(kLowBrakeId);
    m_highBrake = new DigitalInput(kHighBrakeId);

    m_topMotor.restoreFactoryDefaults();
    m_topMotor.setIdleMode(IdleMode.kBrake);
    m_topMotor.setInverted(true);
    m_topMotor.setSmartCurrentLimit((int)kIntakeLimits.supplyLimit());
    m_topMotor.clearFaults();
    
    m_encoder = m_topMotor.getEncoder();
    m_encoder.setPosition(0.0);

    m_pidController = m_topMotor.getPIDController();
    m_pidController.setP(0.2);
    m_pidController.setI(0.0);
    m_pidController.setD(0.0);
    m_pidController.setOutputRange(-1, 1);

    m_pidController.setSmartMotionMaxVelocity(1000.0, 0);
    m_pidController.setSmartMotionMinOutputVelocity(-1000.0, 0);
    m_pidController.setSmartMotionMaxAccel(1000.0, 0);
    m_pidController.setSmartMotionAllowedClosedLoopError(2.0, 0);
    
    m_bottomMotor.restoreFactoryDefaults();
    m_bottomMotor.setIdleMode(IdleMode.kBrake);
    m_bottomMotor.setInverted(true);
    m_bottomMotor.setSmartCurrentLimit((int)kIntakeLimits.supplyLimit());
    m_bottomMotor.clearFaults();

    m_topMotor.burnFlash();
    m_bottomMotor.burnFlash();
  }

  @Override
  public void periodic() {
    Leds.getInstance().NoteDetected = highBrake();
    if(highBrake() && !m_lastBeamState) {
      m_noteSeenPosition = m_encoder.getPosition();
    }
    m_lastBeamState = highBrake();
  }

  public void noteToPosition() {
    m_pidController.setReference(m_noteSeenPosition - 2000, ControlType.kSmartMotion);
  }

  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  public void setVelocity(double velocity) {
    m_pidController.setReference(velocity, ControlType.kSmartVelocity);
  }

  public void setPercent(double percent) {
    setPercent(percent, percent);
  }

  public void setPercent(double top, double bottom) {
    m_topMotor.set(top);
    m_bottomMotor.set(bottom);
  }

  public void stop() {
    m_topMotor.set(0);
    m_bottomMotor.set(0);
  }

  public boolean highBrake() {
    return !m_highBrake.get();
  }

  public boolean lowBrake() {
    return !m_lowBrake.get();
  }

  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Arm");
    
    if(Constants.Dashboard.kSendStates) {
      builder.addBooleanProperty("High Brake", this::highBrake, null);
      builder.addBooleanProperty("Low Brake", this::lowBrake, null);
      builder.addDoubleProperty("Velocity", this::getVelocity, null);
      builder.addDoubleProperty("Position", ()->{return m_encoder.getPosition();}, null);
      builder.addDoubleProperty("Note Seen", ()->{return m_noteSeenPosition;}, null);
    }

  }
}
