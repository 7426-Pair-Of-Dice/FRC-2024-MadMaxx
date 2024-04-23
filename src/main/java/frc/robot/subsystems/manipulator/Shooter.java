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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  private final TalonFX m_topMotor;
  private final TalonFX m_bottomMotor;

  private final MotionMagicVelocityVoltage m_output;
  
  private final XboxController m_driver = OI.getInstance().getDriver().getHID();

  private static double m_velocity;
  public static double m_setVelocity;

  private static final double kThreshold = 4.0;

  private Shooter() {
    m_topMotor = new TalonFX(kShooterConfig.topId());
    m_bottomMotor = new TalonFX(kShooterConfig.bottomId());

    m_output = new MotionMagicVelocityVoltage(0, 0, false, 0, 0, false, false, false);
        
    configMotor(m_topMotor.getConfigurator());
    configMotor(m_bottomMotor.getConfigurator());

    m_topMotor.optimizeBusUtilization();
    m_bottomMotor.optimizeBusUtilization();

    m_topMotor.getVelocity().setUpdateFrequency(50);

    m_output.UpdateFreqHz = 0;

    m_velocity = 0.0;
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

    // Apply configuration
    config.apply(newConfig, 0.050);
  }

  /**
   * Sets the velocity on the top and bottom shooter motors.
   * @param rps Target velocity to drive toward in rotations per second.
   */
  public void setVelocity(double rps) {
    m_velocity = rps;

    m_topMotor.setControl(m_output.withVelocity(m_velocity).withAcceleration(90));
    m_bottomMotor.setControl(m_output.withVelocity(m_velocity).withAcceleration(90));
  }
  
  /**
   * Stops both shooter motors.
   */
  public void stop() {
    m_velocity = 0.0;
    m_driver.setRumble(RumbleType.kBothRumble, 0.0);

    m_topMotor.setControl(m_output.withVelocity(m_velocity));
    m_bottomMotor.setControl(m_output.withVelocity(m_velocity));
  }

  /**
   * @return Velocity of the top motor in mechanism rotations per second.
   */
  public double getVelocity() {
    return m_topMotor.getVelocity().getValueAsDouble();
  }

  /**
   * @return The error between the actual velocity versus the desired velocity.
   */
  public double getError() {
    return m_velocity - getVelocity();
  }

  /**
   * @return True if the current error is within a desired threshold.
   */
  public boolean velocityReached() {
    return Math.abs(getError()) < kThreshold;
  }


  @Override
  public void periodic() {
    // True if the shooter has a non-zero reference, in teleop, and is within threshold.
    boolean shooterReady = m_velocity > 0
                          && DriverStation.isTeleop()
                          && velocityReached();

    // Update Led flags to display shooter status.
    Leds.getInstance().ShooterRunning = m_velocity > 0;
    Leds.getInstance().ShooterVelocityPercentage = getVelocity() / m_velocity;
    Leds.getInstance().ShooterReady = shooterReady;

    // Rumble the driver controller when the shooter is ready.
    m_driver.setRumble(RumbleType.kBothRumble, shooterReady ? 0.5 : 0.0);
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
  }
}
