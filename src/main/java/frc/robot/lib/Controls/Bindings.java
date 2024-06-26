// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.Controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The bindings class is used to keep all DriverStation inputs organized
 * in an effort to keep our inputs easy to read and modify on the fly.
 */
public class Bindings {
    private static OI m_oi = OI.getInstance();
    private static CommandXboxController m_drive = m_oi.getDriver();
    private static CommandXboxController m_operator = m_oi.getOperator();

    public static class Auto {
        public static Trigger Aim = m_operator.leftBumper();
        public static Trigger Rev = m_operator.rightTrigger();
        public static Trigger Center = m_drive.leftTrigger();
    }

    public static class Setpoint {
        public static Trigger IntakeGround = m_operator.rightBumper();
        public static Trigger Speaker = m_operator.leftTrigger();
        public static Trigger Amp = m_operator.x();
        public static Trigger Podium = m_operator.y();
        public static Trigger Reset = m_operator.start();
    }

    public final class Climber {
        public static Trigger In = m_operator.povUp();
        public static Trigger Out = m_operator.povDown();
    }

    public final class Drivetrain {
        public static Trigger Brake = m_drive.a();
        public static Trigger Reorient = m_drive.leftBumper().or(m_drive.povLeft());
    }

    public final class Shooter {
        public static Trigger Out = m_operator.povRight();
    }

    public final class Intake {
        public static Trigger In = m_drive.rightTrigger().or(m_drive.rightBumper());
        public static Trigger Beam = m_operator.a();
        public static Trigger Out = m_operator.b();
    }
}
