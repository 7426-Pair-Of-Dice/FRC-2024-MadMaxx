// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.body.BodyConstants.Setpoint;
import frc.robot.subsystems.body.Arm;
import frc.robot.subsystems.body.Arm.ArmState;
import frc.robot.subsystems.manipulator.Intake;
import frc.robot.subsystems.manipulator.Shooter;

import static frc.robot.commands.Manipulator.secureIntake;

/**
 * Register commands is used to keep pathplanner's named commands organized
 * and separate from the rest of the code. I definitely enjoy the separation
 * of this approach, but major improvements can be made to optimize the commands
 * themselves. 
 */
public class RegisterCommands {
    private final Arm m_arm;
    private final Intake m_intake;
    private final Shooter m_shooter;

    public RegisterCommands() {
        m_arm = Arm.getInstance();
        m_intake = Intake.getInstance();
        m_shooter = Shooter.getInstance();
    }

    public void register() {
        // Shooter and Intake commands
        NamedCommands.registerCommand(
            "Stop Manipulator", 
            new SequentialCommandGroup(
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0),
                new InstantCommand(()->{m_arm.setState(ArmState.Setpoint);},m_arm)
            )
        );

        // Shooter Commands
        NamedCommands.registerCommand(
            "Rev Slot 1", 
            runShooterStateless(80.0)
        );

        // Intake Commands
        NamedCommands.registerCommand(
            "Secure Intake",
            secureIntake().withTimeout(5.0)
        );

        NamedCommands.registerCommand(
            "Intake In",
            runIntake(1.0, 5.0)
        );

        // Setpoint Commands
        NamedCommands.registerCommand("Setpoint Idle", new UpdateSetpoint(Setpoint.Idle));
        NamedCommands.registerCommand("Setpoint Intake In", intakeSetpoint(false));
        NamedCommands.registerCommand("Setpoint Intake Out", intakeSetpoint(true));

        NamedCommands.registerCommand("Beam to Idle", new SequentialCommandGroup(
            new WaitUntilCommand(()->{return m_intake.lowBrake();}),
            new UpdateSetpoint(Setpoint.Idle)
        ));

        // Group Commands
        NamedCommands.registerCommand(
            "Score Preload",
            new SequentialCommandGroup(
                new UpdateSetpoint(Setpoint.Slot9),
                runShooter(80.0, 1.25),
                runIntake(1.0, 0.25),
                new UpdateSetpoint(Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );
        
        NamedCommands.registerCommand(
            "Score Slot 1",
            new SequentialCommandGroup(
                new UpdateSetpoint(Setpoint.Slot9),
                runShooter(80.0, 1.0),
                runIntake(1.0, 0.08),
                new UpdateSetpoint(Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );
        
        NamedCommands.registerCommand(
            "Score Slot 2",
            new SequentialCommandGroup(
                new UpdateSetpoint(Setpoint.Slot6),
                runIntake(1.0, 0.2),
                new UpdateSetpoint(Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        NamedCommands.registerCommand(
            "Score Slot 3",
            new SequentialCommandGroup(
                new UpdateSetpoint(Setpoint.Slot6),
                runIntake(1.0, 0.5),
                new UpdateSetpoint(Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );
    }


    
    private Command intakeSetpoint(boolean out) {
        if(out) {
            return new SequentialCommandGroup(
                new UpdateSetpoint(Setpoint.IntakeGround)
            );
        } else {
            return new SequentialCommandGroup(
                new UpdateSetpoint(Setpoint.Idle)
            );
        }
    }

    private Command runShooterStateless(double rps) {
        if(rps > 0) {
            return new RunCommand(()->{m_shooter.setVelocity(rps);}, m_shooter);
        } else return new InstantCommand(()->{m_shooter.stop();}, m_shooter);
    }


    private Command runShooter(double rps, double timeout) {
        if(rps > 0) {
            return new ParallelDeadlineGroup(
                new WaitCommand(timeout),
                new RunCommand(()->{m_shooter.setVelocity(rps);}, m_shooter)
            );
        } else return new InstantCommand(()->{m_shooter.stop();}, m_shooter);
    }

    private Command runIntake(double power, double timeout) {
        if(power > 0) {
            return new ParallelDeadlineGroup(
                new WaitCommand(timeout),
                new RunCommand(()->{m_intake.setPercent(power);}, m_intake)
            );
        } else return new InstantCommand(()->{m_intake.stop();}, m_intake);
    }

}
