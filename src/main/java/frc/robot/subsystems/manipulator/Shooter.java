// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotState;
import frc.robot.lib.Controls.OI;
import frc.robot.shared.Constants;
import frc.robot.subsystems.leds.Leds;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

public class Shooter extends SubsystemBase {
  private static Shooter m_instance;
  
  public static Shooter getInstance() {
    if(m_instance == null) m_instance = new Shooter();
    return m_instance;
  }

  // The actual motor objects, nothing crazy about these
  private final TalonFX m_topMotor = new TalonFX(kShooterConfig.topId());
  private final TalonFX m_bottomMotor = new TalonFX(kShooterConfig.bottomId());
  private final XboxController m_driveController = OI.getInstance().getDriver().getHID();

  // This seems to be what you would use to determine the power sent to the motor
  private final MotionMagicVelocityVoltage m_topOutput = new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);
  private final MotionMagicVelocityVoltage m_bottomOutput = new MotionMagicVelocityVoltage(0, 0, true, 0, 0, false, false, false);

  private static double m_velocity;
  public static double m_setVelocity;

  private static double m_threshold = 4.0;

  private static double m_customRPS = 70.0; 
  

  /** Creates a new Shooter. */
  private Shooter() {
  
    configMotor(m_topMotor.getConfigurator());
    configMotor(m_bottomMotor.getConfigurator());

    m_topMotor.optimizeBusUtilization();
    m_bottomMotor.optimizeBusUtilization();

    m_topMotor.getVelocity().setUpdateFrequency(50);

    m_topOutput.UpdateFreqHz = 0;
    m_bottomOutput.UpdateFreqHz = 0;

    m_velocity = 0.0;
  }

  private static void configMotor(TalonFXConfigurator config) {
    var newConfig = new TalonFXConfiguration();

    // Configure idle mode and polarity
    var output = newConfig.MotorOutput;
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    // Set max voltage
    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 16;
    voltage.PeakReverseVoltage = -16;

    // Set current limits
    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kShooterLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kShooterLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    // Configure PID in Slot 0
    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = kShooterGains.kP();
    slot0.kI = kShooterGains.kI();
    slot0.kD = kShooterGains.kD();
    slot0.kV = kShooterGains.kV();
    
    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    motionMagic.MotionMagicAcceleration = 160.0;
    motionMagic.MotionMagicCruiseVelocity = 320.0;
    motionMagic.MotionMagicJerk = 1600.0;

    config.apply(newConfig, 0.050);
  }


  public void setMotors(double rps) {
    m_velocity = rps;

    m_topMotor.setControl(m_topOutput.withVelocity(m_velocity).withAcceleration(90));
    m_bottomMotor.setControl(m_bottomOutput.withVelocity(m_velocity).withAcceleration(90));
  }

  public double getError() {
    return m_velocity - getVelocity();
  }

  public double getVelocity() {
    return m_topMotor.getVelocity().getValueAsDouble();
  }

  public boolean velocityReached() {
    return Math.abs(getError()) < m_threshold;
  }

  public void stop() {
    m_velocity = 0.0;
    m_driveController.setRumble(RumbleType.kBothRumble, 0.0);
    m_topMotor.setControl(m_topOutput.withVelocity(m_velocity));
    m_bottomMotor.setControl(m_bottomOutput.withVelocity(m_velocity));
  }

  @Override
  public void periodic() {
    
    Leds.getInstance().ShooterRunning = m_velocity > 0;
    Leds.getInstance().ShooterVelocityPercentage = getVelocity() / m_velocity;
    Leds.getInstance().ShooterReady = false;

    if(m_velocity > 0 && RobotContainer.robotStateSupplier.get() == RobotState.Teleop && getVelocity() > (m_velocity - 5)) {
      Leds.getInstance().ShooterReady = true;
      m_driveController.setRumble(RumbleType.kBothRumble, 0.5);
    } else {
      m_driveController.setRumble(RumbleType.kBothRumble, 0.0);
    }


  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    if(Constants.Dashboard.kSendErrors) {
      builder.addDoubleProperty("Error", this::getError, null);
    }
    
    if(Constants.Dashboard.kSendReferences) {
      builder.addDoubleProperty("Reference", ()->{return m_velocity;}, null);
    }

    if(Constants.Dashboard.kSendStates) {
      builder.addDoubleProperty("Velocity", this::getVelocity, null);
      builder.addBooleanProperty("Shooter Locked", this::velocityReached, null);
    }

    if(Constants.Dashboard.kSendDebug) {
      builder.addDoubleProperty("Custom RPS", ()->{return m_customRPS;}, (double value)->{m_customRPS=value;});
    }
  }
}
