// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.body.BodyConstants.Setpoint;
import frc.robot.shared.Limelight;
import frc.robot.shared.LimelightHelpers;
import frc.robot.subsystems.body.Arm;
import frc.robot.subsystems.body.Elevator;
import frc.robot.subsystems.body.Arm.ArmState;
import frc.robot.subsystems.manipulator.Intake;
import frc.robot.subsystems.manipulator.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

import static frc.robot.commands.Manipulator.*;

/** Add your docs here. */
public class RegisterCommands {
    private final Arm m_arm;
    private final Elevator m_elevator;
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Swerve m_drivetrain;

    private final SwerveRequest.RobotCentric m_robotCentric;

    public Supplier<Double> limeAim = () -> {
        double kP = 0.035;
        return LimelightHelpers.getTX("limelight-april") * kP * SwerveConstants.kMaxAngularRate * -1.0;
    };
    
    public Command limelight_aim() {
        return m_drivetrain.applyRequest(()->m_robotCentric.withRotationalRate(limeAim.get()));
    }

    public RegisterCommands(Arm arm, Elevator elevator, Intake intake, Shooter shooter, Swerve drivetrain) {
        m_arm = arm;
        m_elevator = elevator;
        m_intake = intake;
        m_shooter = shooter;
        m_drivetrain = drivetrain;

        m_robotCentric = new SwerveRequest.RobotCentric();
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
        NamedCommands.registerCommand(
            "Eject Forwards", 
            new SequentialCommandGroup(
                runShooter(30.0, 0.25),
                runIntake(1.0, 0.25),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );
        NamedCommands.registerCommand(
            "Eject Backwards", 
            new SequentialCommandGroup(
                runShooter(-30.0, 0.25),
                runIntake(-1.0, 0.25),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        // Shooter Commands
        NamedCommands.registerCommand(
            "Rev Speaker", 
            runShooter(80.0, 1.25));
        NamedCommands.registerCommand(
            "Rev Podium", 
            runShooter(80.0, 2.0));
        NamedCommands.registerCommand(
            "Rev Slot 1", 
            runShooterStateless(80.0)
        );
        NamedCommands.registerCommand(
            "Rev Slot 2", 
            runShooter(0.0, 0.0)
        );
        NamedCommands.registerCommand(
            "Rev Slot 3", 
            runShooter(85.0, 2.0)
        );
        NamedCommands.registerCommand(
            "Stop Shooter",
            runShooter(0.0, 0.0)
        );

        // Intake Commands
        NamedCommands.registerCommand(
            "Run Intake", 
            runIntake(1.0, 0.25));
        NamedCommands.registerCommand(
            "Run Beam Brake", 
            runBeamBrake(1.0).withTimeout(5.0)
        );
        NamedCommands.registerCommand(
            "Run Beam Brake 2", 
            runBeamBrake2(1.0).withTimeout(5.0)
        );
        NamedCommands.registerCommand(
            "Run Beam Brake Auto", 
            runBeamBrake(0.50).withTimeout(5.0)
        );
        NamedCommands.registerCommand(
            "Secure Intake",
            secureIntake().withTimeout(5.0)
        );
        NamedCommands.registerCommand(
            "Secure Note", 
            runBeamSafety()
        );
        NamedCommands.registerCommand(
            "Slow Secure", 
            slowSecure()
        );
        NamedCommands.registerCommand(
            "Secure Note Timeout", 
            runBeamSafety().withTimeout(0.25)
        );
        NamedCommands.registerCommand(
            "Stop Intake", 
            runIntake(0.0, 0.0)
        );

        NamedCommands.registerCommand("Score Stateless Card Suits", limelight_aim());

        NamedCommands.registerCommand("Limelight Center", limelight_aim());

        // Setpoint Commands
        NamedCommands.registerCommand("Setpoint Idle", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle));
        NamedCommands.registerCommand("Setpoint Speaker", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Speaker));
        NamedCommands.registerCommand("Setpoint Podium", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Podium));
        NamedCommands.registerCommand("Setpoint Slot 1", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot1));
        NamedCommands.registerCommand("Setpoint Slot 2", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot2));
        NamedCommands.registerCommand("Setpoint Slot 3", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot3));
        NamedCommands.registerCommand("Setpoint Slot 4", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot4));
        NamedCommands.registerCommand("Setpoint Slot 5", new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot5));
        NamedCommands.registerCommand("Setpoint Intake In", intakeSetpoint(false));
        NamedCommands.registerCommand("Setpoint Intake Out", intakeSetpoint(true));

        NamedCommands.registerCommand("Beam to Idle", new SequentialCommandGroup(
            new WaitUntilCommand(()->{return m_intake.lowBrake();}),
            new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle)
        ));
        NamedCommands.registerCommand("Auto Aim", new AutoAim(true));

        // Group Commands
        NamedCommands.registerCommand(
            "Score Preload",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot9),
                runShooter(80.0, 1.0),
                runIntake(1.0, 0.08),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        NamedCommands.registerCommand(
            "Score Stateless",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Speaker),
                runIntake(1.0, 0.08),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        NamedCommands.registerCommand(
            "Score Stateless Modified",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot5),
                new WaitCommand(0.05),
                runIntake(1.0, 0.2),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );
        
        NamedCommands.registerCommand(
            "Score Stateless Modified Again The Sequel",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot8),
                new WaitCommand(0.05),
                runIntake(1.0, 0.2),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        NamedCommands.registerCommand(
            "Score Stateless Modified Again The Sequel Remastered",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot8),
                new WaitCommand(0.1),
                runIntake(1.0, 0.2),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        // NamedCommands.registerCommand(
        //     "Score with Limelight",
        //     new SequentialCommandGroup(
        //         // could be in rev limelight
        //         new InstantCommand(()->{
        //             m_arm.setState(ArmState.ClosedLoop);
        //           }, m_arm),
        //         runShooter(Limelight.calculation.rps(), 2),
        //         // end of rev limelight
        //         // new WaitCommand(1),
        //         runIntake(1.0, 0.2),
        //         new InstantCommand(()->{
        //             m_arm.setState(ArmState.Setpoint);
        //           }, m_arm),
        //         new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
        //         runShooter(0.0, 0.0),
        //         runIntake(0.0, 0.0)
        //     )
        // );

        NamedCommands.registerCommand(
            "Score Stateless Modified Again Again",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot6),
                runIntake(1.0, 0.2),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        NamedCommands.registerCommand(
            "Score Stateless Modified Again Again 2",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot10),
                new WaitCommand(1.0),
                runIntake(1.0, 0.2),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );

        NamedCommands.registerCommand(
            "Score Stateless Modified Again",
            new SequentialCommandGroup(
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Slot6),
                new WaitCommand(0.1),
                runIntake(1.0, 0.14),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle),
                runShooter(0.0, 0.0),
                runIntake(0.0, 0.0)
            )
        );


        // Old Commands (Only used in Card Suits at the moment, need to update)
        NamedCommands.registerCommand("startShooter", new SequentialCommandGroup(
            runShooter(60.0, 20.0)
        ));

        NamedCommands.registerCommand("shootNotePhodium", new SequentialCommandGroup(
            new UpdateSetpoint(m_arm, m_elevator, Setpoint.AutoPhodium),
            new WaitCommand(1),
            runIntake(1.0, 0.25)
        )); 

        NamedCommands.registerCommand("shootNotePhodium2", new SequentialCommandGroup(
            new UpdateSetpoint(m_arm, m_elevator, Setpoint.Speaker),
            new WaitCommand(1),
            runIntake(1.0, 0.25)
        )); 
    }


    private Command runBeamSafety() {
        return new SequentialCommandGroup(
            new RunCommand(()->{m_intake.setPercent(1.0);}, m_intake)
            .until(()->{return m_intake.highBrake();}),
            new ParallelDeadlineGroup(
                new WaitCommand(0.075),
                new RunCommand(()->{m_intake.setPercent(-0.2);}, m_intake)
            ),
            new RunCommand(()->{m_intake.stop();}, m_intake)
        );
    }

    // Run Safety 2.0 - Modified Slightly for Comp

    private Command runBeamBrake2(double power) {
        return new SequentialCommandGroup(
            new RunCommand(()->{m_intake.setPercent(1.0);}, m_intake)
            .until(()->{return m_intake.highBrake();}),
            //new RunCommand(()->{m_intake.setPercent(-0.4);}, m_intake)
            // .withTimeout(0.075),
            new RunCommand(()->{m_intake.stop();}, m_intake)
        );
    }
    
    private Command intakeSetpoint(boolean out) {
        if(out) {
            return new SequentialCommandGroup(
                // new WaitCommand(0.25).deadlineWith(
                //     new UpdateSetpoint(m_arm, m_elevator, Setpoint.BumperOut, 0.0, 0.125)
                // ),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.IntakeGround)
            );
        } else {
            return new SequentialCommandGroup(
                // new WaitCommand(0.25).deadlineWith(
                //     new UpdateSetpoint(m_arm, m_elevator, Setpoint.BumperIn, 0.0, 0.25)
                // ),
                new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle)
            );
        }
    }


    private Command runShooterStateless(double rps) {
        if(rps > 0) {
            return new RunCommand(()->{m_shooter.setMotors(rps);}, m_shooter);
        } else return new InstantCommand(()->{m_shooter.stop();}, m_shooter);
    }


    private Command runShooter(double rps, double timeout) {
        if(rps > 0) {
            return new ParallelDeadlineGroup(
                new WaitCommand(timeout),
                new RunCommand(()->{m_shooter.setMotors(rps);}, m_shooter)
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

    private Command runBeamBrake(double power) {
        return new SequentialCommandGroup(
            new RunCommand(()->{m_intake.setPercent(1.0);}, m_intake)
            .until(()->{return m_intake.highBrake();}),
            new RunCommand(()->{m_intake.setPercent(-0.4);}, m_intake)
            .withTimeout(0.075),
            new RunCommand(()->{m_intake.noteToPosition();}, m_intake)
        );
    }

    private Command slowSecure() {
        return new SequentialCommandGroup(
            new RunCommand(()->{m_intake.setPercent(0.4);}, m_intake)
            .until(()->{return m_intake.highBrake();}),
            new RunCommand(()->{m_intake.stop();}, m_intake)
        );
    }
   
}
