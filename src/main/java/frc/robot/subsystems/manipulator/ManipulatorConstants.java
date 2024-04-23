// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

public class ManipulatorConstants {
    public static final MotorSetup kIntakeConfig = new MotorSetup(23, 24);
    public static final Limits kIntakeLimits = new Limits(0.0, 30.0);

    public static final MotorSetup kShooterConfig = new MotorSetup(15, 16);
    public static final Limits kShooterLimits = new Limits(80.0, 40.0);
    public static final Gains kShooterGains = new Gains(0.0,0.0,0.0,0.0,0.12,0.0);

    public record MotorSetup(int topId, int bottomId) {};
    public record Limits(double statorLimit, double supplyLimit) {};
    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
