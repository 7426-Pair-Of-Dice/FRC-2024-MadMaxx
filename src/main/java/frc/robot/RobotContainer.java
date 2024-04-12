// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DebugShot;
import frc.robot.commands.RegisterCommands;
import frc.robot.commands.UpdateSetpoint;
import frc.robot.lib.Controls.Bindings;
import frc.robot.lib.Controls.OI;
import frc.robot.shared.Constants;
import frc.robot.shared.Limelight;
import frc.robot.subsystems.body.BodyConstants.Setpoint;
import frc.robot.subsystems.body.*;
import frc.robot.subsystems.body.Arm.ArmState;
import frc.robot.subsystems.manipulator.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.leds.Leds;

import static frc.robot.commands.Manipulator.*;


public class RobotContainer {
  
  public enum Alliance {
    BLUE,
    RED
  }

  public enum RobotState {
    Teleop,
    Autonomous,
    Test,
    Disabled
  }

  private static RobotState m_robotState;
  public static Supplier<RobotState> robotStateSupplier;

  private static Timer m_matchTimer;
  
  // Declaring Input Methods
  private static OI m_oi;
  
  private final Swerve m_drivetrain;

  private final SwerveRequest.FieldCentric drive;

  private final SwerveRequest.SwerveDriveBrake brake;
  private final SwerveRequest.RobotCentric m_robotCentric = new SwerveRequest.RobotCentric();
  private final SwerveRequest.PointWheelsAt point;
  private final Telemetry logger = new Telemetry(SwerveConstants.kMaxSpeed);

  // Declaring Subsystems
  private final Arm m_arm;
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Climber m_climber;
  private final Shooter m_shooter;
  private final Leds m_leds;

  // Setpoints
  private final Supplier<UpdateSetpoint> m_setpointIdle;
  private final ParallelCommandGroup m_setpointIntakeGround;
  private final ParallelCommandGroup m_setpointSpeaker;
  private final ParallelCommandGroup m_setpointAmp;
  private final ParallelCommandGroup m_setpointTrap;
  private final ParallelCommandGroup m_setpointPodium;

  private final ParallelDeadlineGroup m_bumperClearOut;
  private final ParallelDeadlineGroup m_bumperClearIn;
  private final ParallelDeadlineGroup m_trapClearOut;
  private final ParallelDeadlineGroup m_trapClearIn;

  // Commands
  private final RegisterCommands m_registerCommands;

  private final Supplier<Command> m_intakeUntilNote;
  private final Supplier<Command> m_secureNote;
  
  private final Supplier<RunCommand> m_climberIn;
  private final Supplier<RunCommand> m_climberOut;
  
  private final RunCommand m_stopClimber;

  // Paths
  public SendableChooser<Command> m_autoSelector;
  private static SendableChooser<Alliance> m_allianceSelector;
  
  public Command limelight_aim() {
    return m_drivetrain.applyRequest(() -> drive.withRotationalRate(Limelight.calculateCentering())
        );
  }

  public RobotContainer() {
    // Initialize Input Methods
    m_oi = OI.getInstance();

    robotStateSupplier = () -> m_robotState;

    // Initialize Subsystems
    m_arm = Arm.getInstance();
    m_elevator = Elevator.getInstance();
    m_intake = Intake.getInstance();
    m_climber = Climber.getInstance();
    m_shooter = Shooter.getInstance();
    m_leds = Leds.getInstance();
    // m_led = new LED();
    
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

    m_intakeUntilNote = () -> {
      return new SequentialCommandGroup(
        runIntake()
        .until(()->{return m_intake.highBrake();}),
        runIntake(-0.2).withTimeout(0.075),
        stopIntake()
      );
    };

    m_secureNote = () -> {
      return new SequentialCommandGroup(
        new RunCommand(()->{m_intake.setPercent(-0.2);}, m_intake)
        .until(()->{return !m_intake.highBrake();})
        .withTimeout(0.075),
        stopIntake()
      );
    };

    m_climberIn = () -> {
      return new RunCommand(()->{m_climber.setMotor(1.0);}, m_climber);
    };

    m_climberOut = () -> {
      return new RunCommand(()->{m_climber.setMotor(-1.0);}, m_climber);
    };

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
      new RunCommand(()->{m_shooter.setMotors(10.0);}, m_shooter)
    );
    
    m_setpointTrap = new ParallelCommandGroup(
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.Trap),
      runShooter(22.7426)
    );

    m_setpointPodium = new ParallelCommandGroup(
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.Podium, 0.0, 0.1),
      runShooter(80.0)
    );

    m_bumperClearOut = new ParallelDeadlineGroup(
      new WaitCommand(0.175),
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.BumperOut, 0.0, 0.0)
    );
    
    m_bumperClearIn = new ParallelDeadlineGroup(
      new WaitCommand(0.6),
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.BumperIn, 0.0, 0.5)
    );

    m_trapClearOut = new ParallelDeadlineGroup(
      new WaitCommand(0.5),
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.TrapOut, 0.0, 0.125)
    );
    
    m_trapClearIn = new ParallelDeadlineGroup(
      new WaitCommand(0.6),
      new UpdateSetpoint(m_arm, m_elevator, Setpoint.TrapIn, 0.0, 0.25)
    );
    
    // Climber Default
    m_climber.setDefaultCommand(m_stopClimber);

    // Pathplanner
    m_autoSelector = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Autonomous", m_autoSelector);

    m_matchTimer = new Timer();

    configureBindings();
  }

  private void configureBindings() {

    // Swerve Bindings
    Bindings.Auto.Center
      .whileTrue(limelight_aim());

    Bindings.Drivetrain.Break
      .whileTrue(m_drivetrain.applyRequest(() -> brake));

    Bindings.Drivetrain.RoboOrient
      .whileTrue(m_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0, 0))));

    Bindings.Drivetrain.Reorient
      .onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));


    // Intake Bindings
    Bindings.Intake.In
      .whileTrue(runIntake())
      .onFalse(stopIntake());

    Bindings.Intake.Beam
      .whileTrue(secureIntake())
      .onFalse(stopIntake());

    Bindings.Intake.Out
      .whileTrue(new RunCommand(()->{m_intake.setPercent(-0.2);}, m_intake))
      .onFalse(stopIntake());
    
    // Climber Bindings
    Bindings.Climber.In
      .whileTrue(m_climberIn.get())
      .onFalse(m_stopClimber);

    Bindings.Climber.Out
      .whileTrue(m_climberOut.get())
      .onFalse(m_stopClimber);


    // Shooter
    Bindings.Shooter.Out
      .whileTrue(runShooter(40.0))
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

    Bindings.Setpoint.Amp
      .whileTrue(m_setpointAmp)
      .onFalse(m_setpointIdle.get().andThen(stopShooter()));
      
    Bindings.Setpoint.Speaker
      .whileTrue(m_setpointSpeaker)
      .onFalse(m_setpointIdle.get().andThen(stopShooter()));
      
    Bindings.Setpoint.Podium
      .whileTrue(m_setpointPodium)
      .onFalse(m_setpointIdle.get().andThen(stopShooter()));
      
    Bindings.Setpoint.Trap
      .whileTrue(m_trapClearOut.andThen(m_setpointTrap))
      .whileFalse(m_trapClearIn.andThen(m_setpointIdle.get())
                  .andThen(stopShooter())
      );

    Bindings.Setpoint.Reset
        .whileTrue(new ParallelCommandGroup(
          new UpdateSetpoint(m_arm, m_elevator, Setpoint.Reset),
          m_drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_oi.getDriver().getLeftY(), -m_oi.getDriver().getLeftX())))
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
  
  public Alliance getAlliance() {
    return m_allianceSelector.getSelected();
  }

  public static RobotState getRobotState() {
    return m_robotState;
  }

  public void setRobotState(RobotState state) {
      m_robotState = state;

    if (m_robotState == RobotState.Disabled) {
      m_matchTimer.stop();
      m_matchTimer.reset();
    } else if (m_robotState == RobotState.Teleop) {
      m_matchTimer.start();
    }
  }
}
