// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class Leds {
  private static Leds m_instance;
  
  public static Leds getInstance() {
    if(m_instance == null) m_instance = new Leds();
    return m_instance;
  }

  // LED Physical Constants
  private final int Length = 33;
  private final int Port = 0;

  // LED IO
  private final AddressableLED m_leds;
  private final AddressableLEDBuffer m_buffer;

  // Flags used to determine LED states
  public static Color DefaultColor = new Color(228,0,228);
  public boolean NoteDetected = false;
  public boolean ShooterRunning = false;
  public boolean ShooterReady = false;
  public double ShooterVelocityPercentage = 0.0;
  public boolean AutonomousFinished = false;
  public double AutonomousEnd = 0.0;
  
  private Leds() {
    m_leds = new AddressableLED(Port);
    m_buffer = new AddressableLEDBuffer(Length);
    m_leds.setLength(Length);
    m_leds.setData(m_buffer);
    m_leds.start();

  }
  
  public synchronized void periodic() {
    if (DriverStation.isDisabled()) {
      AutonomousFinished = false;
    }

    solid(new Color(0, 0, 0));
    
    if(DriverStation.isAutonomous()) solid(DefaultColor);

    if(NoteDetected) solid(new Color(255, 40, 0));

    if(ShooterRunning) percent(new Color(100, 100, 100), ShooterVelocityPercentage);

    if(ShooterReady) solid(new Color(0, 255, 0));

    if(AutonomousFinished && (Timer.getFPGATimestamp() - AutonomousEnd) < 5.0) {
      solid(new Color(255,255,255));
    }

    if(DriverStation.isDisabled()) solid(DefaultColor);

    m_leds.setData(m_buffer);
  }


  private void percent(Color color, double percentage) {
    int percentLength = (int) Math.floor(percentage * Length);
    if (color == null) return;
    for (int i = 0; i < Length; i++) {
      if(i < percentLength) {
        m_buffer.setLED(i, color);
      } else m_buffer.setLED(i, new Color(0, 0, 0));
    }
  }

  private void solid(Color color) {
    if (color == null) return;
    for (int i = 0; i < Length; i++) {
      m_buffer.setLED(i, color);
    }
  }
}
