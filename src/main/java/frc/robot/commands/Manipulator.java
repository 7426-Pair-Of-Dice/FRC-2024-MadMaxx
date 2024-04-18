package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.manipulator.*;

public class Manipulator {
    private static Intake m_intake = Intake.getInstance();
    private static Shooter m_shooter = Shooter.getInstance();

    // Intake Commands

    public static Command runIntake() {
        return runIntake(1.0);
    }

    public static Command runIntake(double percent) {
        return new RunCommand(()->{m_intake.setPercent(percent);}, m_intake);
    }
    
    public static Command stopIntake() {
        return new InstantCommand(()->{m_intake.stop();}, m_intake);
    }

    public static Command secureIntake() {
        return secureIntake(false);
    }

    public static Command secureIntake(boolean reverseShooter) {
        Command reverseManipulator = reverseShooter ? new ParallelCommandGroup(
            runIntake(-0.1),
            runShooter(-10.0)
        ) : runIntake(-0.1);

        Command stopManipulator = reverseShooter ? stopManipulator() : stopIntake();

        return new SequentialCommandGroup(
            runIntake(1.0).until(() -> m_intake.lowBrake()), // Run until the low beam brake is triggered
            runIntake(0.6).until(() -> m_intake.highBrake()), // Run at a slower speed until the high beam brake is triggered
            reverseManipulator.until(() -> !m_intake.highBrake()),
            stopManipulator
        );
    }

    // Shooter Commands

    public static Command runShooter(double rps) {
        return new RunCommand(()->{m_shooter.setMotors(rps);}, m_shooter);
    }
    
    public static Command stopShooter() {
        return new RunCommand(()->{m_shooter.stop();}, m_shooter);
    }

    // Group Commands

    public static Command ejectNote() {
        return new ParallelCommandGroup(
            new RunCommand(()->{m_intake.setPercent(-0.4);}, m_intake),
            new RunCommand(()->{m_shooter.setMotors(-15.0);}, m_shooter)
        );
    }

    public static Command stopManipulator() {
        return new ParallelCommandGroup(
            new RunCommand(()->{m_intake.stop();}, m_intake),
            new RunCommand(()->{m_shooter.stop();}, m_shooter)
        );
    }
    
}
