// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoAim;
import frc.robot.commands.RegisterCommands;
import frc.robot.commands.UpdateSetpoint;
import frc.robot.lib.Controls.Bindings;
import frc.robot.lib.Controls.OI;
import frc.robot.shared.Constants;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.BodyConstants.Setpoint;
import frc.robot.subsystems.body.*;
import frc.robot.subsystems.manipulator.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.climber.Climber;

import static frc.robot.commands.Manipulator.*;


public class RobotContainer {

  // Declaring Input Methods
  private static OI m_oi;
  
  private final Swerve m_drivetrain;

  private final SwerveRequest.FieldCentric drive;

  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.PointWheelsAt point;
  private final Telemetry logger = new Telemetry(SwerveConstants.kMaxSpeed);

  // Declaring Subsystems
  private final Arm m_arm;
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Climber m_climber;
  private final Shooter m_shooter;

  // Setpoints
  private final Supplier<UpdateSetpoint> m_setpointIdle;
  private final ParallelCommandGroup m_setpointIntakeGround;
  private final ParallelCommandGroup m_setpointSpeaker;
  private final ParallelCommandGroup m_setpointAmp;
  private final ParallelCommandGroup m_setpointPodium;
  
  // Commands
  private final RunCommand m_stopClimber;

  // Autonomous
  private final RegisterCommands m_registerCommands;
  public SendableChooser<Command> m_autoSelector;
  
  public Command limelight_aim() {
    return m_drivetrain.applyRequest(() -> drive.withRotationalRate(Limelight.calculateCentering()).withVelocityX(0).withVelocityY(0));
  }

  public RobotContainer() {
    // Initialize Input Methods
    m_oi = OI.getInstance();

    // Initialize Subsystems
    m_arm = Arm.getInstance();
    m_elevator = Elevator.getInstance();
    m_intake = Intake.getInstance();
    m_climber = Climber.getInstance();
    m_shooter = Shooter.getInstance();
    
    // Phoenix Swerve Initialization
    m_drivetrain = SwerveConstants.DriveTrain;

    drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.kMaxSpeed * SwerveConstants.kDeadband)
      .withRotationalDeadband(SwerveConstants.kMaxAngularRate * SwerveConstants.kDeadband)
      .withDriveRequestType(DriveRequestType.Velocity);

    brake = new SwerveRequest.SwerveDriveBrake();
    point = new SwerveRequest.PointWheelsAt();
    
    m_registerCommands = new RegisterCommands();
    m_registerCommands.register();
    
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(() -> drive.withVelocityX(-m_oi.getDriver().getLeftY() * SwerveConstants.kMaxSpeed)
            .withVelocityY(-m_oi.getDriver().getLeftX() * SwerveConstants.kMaxSpeed)
            .withRotationalRate(-m_oi.getDriver().getRightX() * SwerveConstants.kMaxAngularRate)
        ));
    
    m_stopClimber = new RunCommand(() -> m_climber.stop(), m_climber);

    m_setpointIdle = () -> {
      return new UpdateSetpoint(m_arm, m_elevator, Setpoint.Idle);
   
    };

    m_setpointIntakeGround = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new UpdateSetpoint(m_arm, m_elevator, Setpoint.IntakeGround),
        new WaitUntilCommand(()->m_intake.lowBrake()),
        m_setpointIdle.get()
      ),
      secureIntake()
    );
    
    m_setpointSpeaker = new ParallelCommandGroup(
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.Speaker, 0.0, 0.1),
      runShooter(70.0)
    
    );

    m_setpointAmp = new ParallelCommandGroup(
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.Amp, 0.0, 0.1),
      new RunCommand(()->{m_shooter.setVelocity(10.0);}, m_shooter)
    );

    m_setpointPodium = new ParallelCommandGroup(
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.Podium, 0.0, 0.1),
      runShooter(80.0)
    );
    
    // Climber Default
    m_climber.setDefaultCommand(m_stopClimber);

    // Pathplanner
    m_autoSelector = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", m_autoSelector);

    configureBindings();
  }

  private void configureBindings() {

    // Swerve Bindings
    Bindings.Auto.Center
      .whileTrue(limelight_aim());

    Bindings.Drivetrain.Brake
      .whileTrue(m_drivetrain.applyRequest(() -> brake));

    Bindings.Drivetrain.Reorient
      .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));


    // Intake Bindings
    Bindings.Intake.In
      .whileTrue(runIntake())
      .onFalse(stopIntake());

    Bindings.Intake.Beam
      .whileTrue(secureIntake(true))
      .onFalse(stopIntake());

    Bindings.Intake.Out
      .whileTrue(new RunCommand(()->{m_intake.setPercent(-0.2);}, m_intake))
      .onFalse(stopIntake());
    
    // Climber Bindings
    Bindings.Climber.In
      .whileTrue(new RunCommand(()->{m_climber.setMotor(1.0);}, m_climber))
      .onFalse(m_stopClimber);

    Bindings.Climber.Out
      .whileTrue(new RunCommand(()->{m_climber.setMotor(-1.0);}, m_climber))
      .onFalse(m_stopClimber);


    // Shooter
    Bindings.Shooter.Out
      .whileTrue(runShooter(45.0))
      .onFalse(stopShooter());

    // Commands
    Bindings.Setpoint.IntakeGround
      .whileTrue(
        // m_bumperClearOut
        // .andThen(
          m_setpointIntakeGround
        )
      .whileFalse(
        m_setpointIdle.get()
        .andThen(stopIntake())
      );
      
    
    Bindings.Auto.Aim
      .whileTrue(new AutoAim());

    Bindings.Auto.Rev
      .whileTrue(runShooter(70.0))
      .onFalse(stopShooter());

    Bindings.Setpoint.Amp
      .whileTrue(m_setpointAmp)
      .onFalse(m_setpointIdle.get().andThen(stopShooter()));
      
    Bindings.Setpoint.Speaker
      .whileTrue(m_setpointSpeaker)
      .onFalse(m_setpointIdle.get().andThen(stopShooter()));
      
    Bindings.Setpoint.Podium
      .whileTrue(m_setpointPodium)
      .onFalse(m_setpointIdle.get().andThen(stopShooter()));
      
    Bindings.Setpoint.Reset
        .whileTrue(new ParallelCommandGroup(
          new UpdateSetpoint(m_arm, m_elevator, Setpoint.Reset),
          m_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0, 0)))
        ));

    // Dashboard
    SmartDashboard.putData(m_arm);
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(m_intake);
    
    if(Constants.Dashboard.kSendSwerve) {
      m_drivetrain.registerTelemetry(logger::telemeterize);
    }
  }

  public Command getAutonomousCommand() {
    return m_autoSelector.getSelected();
  }
}
