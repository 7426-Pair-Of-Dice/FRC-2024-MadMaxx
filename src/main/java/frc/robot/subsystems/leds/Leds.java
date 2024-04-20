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
  // public Optional<Alliance> ActiveAlliance = DriverStation.getAlliance();
  // public var ActiveAlliance = DriverStation.getAlliance();
  public Color AllianceColor;
  public boolean EmergencyStopped = false;
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

  public class Test {
    
  }

  public synchronized void periodic() {
    // if (DriverStation.getAlliance().isPresent()) {
    //   try {
    //     AllianceColor = ActiveAlliance.get() == DriverStation.Alliance.Blue ? new Color(0, 0, 255) : new Color(255, 0, 0);
    //   } catch(Exception e) {
        AllianceColor = new Color(228,0,228);
    //   }
    // }

    if (DriverStation.isDisabled()) {
      AutonomousFinished = false;
    }

    if (DriverStation.isEStopped()) {
      EmergencyStopped = true;
    }

    solid(new Color(0, 0, 0));
    
    if(DriverStation.isAutonomous()) solid(AllianceColor);
    

    if(NoteDetected) solid(new Color(255, 40, 0));

    if(ShooterRunning) percent(new Color(100, 100, 100), ShooterVelocityPercentage);

    if(ShooterReady) breathe(new Color(0, 255, 0), new Color(0, 248, 0));

    if(AutonomousFinished && (Timer.getFPGATimestamp() - AutonomousEnd) < 5.0) {
      solid(new Color(255,255,255));
    }

    if(DriverStation.isDisabled()) breathe(new Color(228,0,228), AllianceColor, 4.0);

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

  private void breathe(Color c1, Color c2) {
    breathe(c1, c2, 1.0);
  }
  
  private void breathe(Color c1, Color c2, double duration) {
    double x = ((Timer.getFPGATimestamp() % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(new Color(red, green, blue));
  }

}
