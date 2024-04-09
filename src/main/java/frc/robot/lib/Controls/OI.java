// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.Controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.shared.Alert;
import frc.robot.shared.Alert.AlertType;

public class OI {
    private static OI m_instance;

    public static OI getInstance() {
        if(m_instance == null) m_instance = new OI();
        return m_instance;
    }

    private final int m_driveId = 0;
    private final int m_operatorId = 1;

    private static CommandXboxController m_drive;
    private static CommandXboxController m_operator;

    private Alert m_alertDriver;
    private Alert m_alertOperator;

    private String m_alertFormat = "The %s controller is not connected. (Port %d)";

    private OI() {
        m_drive = new CommandXboxController(m_driveId);
        m_operator = new CommandXboxController(m_operatorId);

        m_alertDriver = new Alert(String.format(m_alertFormat, "driver", m_driveId), AlertType.WARNING);
        m_alertOperator = new Alert(String.format(m_alertFormat, "operator", m_operatorId), AlertType.WARNING);
    }

    public CommandXboxController getDriver() {
        return m_drive;
    }

    public CommandXboxController getOperator() {
        return m_operator;
    }

    public void checkConnections() {
        m_alertDriver.set(!DriverStation.isJoystickConnected(m_drive.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(m_drive.getHID().getPort()));

        m_alertOperator.set(!DriverStation.isJoystickConnected(m_operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(m_operator.getHID().getPort()));
    }
}
