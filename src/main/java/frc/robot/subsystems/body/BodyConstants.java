// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.body;

public class BodyConstants {

    public static final ArmConfig kArmConfig = new ArmConfig(17, 18, 21);
    public static final Limits kArmLimits = new Limits(80.0, 45.0, 0.29, 1.0 / 360.0);

    public static final ElevatorConfig kElevatorConfig = new ElevatorConfig(19, 20, 22);
    public static final Limits kElevatorLimits = new Limits(100.0, 45.0, 2.3, -0.05);

    public static final ClimberLimits kClimberLimits = new ClimberLimits(80.0, 45.0);

    /**
     * An enum used to store the information needed for each setpoint relating to the arm and elevator.
     *
     * @param arm The set position for the arm to go to in degrees.
     * @param armFeed The set feed forward for the arm to use in setpoints.
     * @param elevator The set position for the elevator to go to in rotations.
     * @param elevatorFeed The set feed forward for the elevator to use in setpoints.
     * @param elevatorSlot The set PID slot for the elevator Talons to use in setpoints.
     */
    public static enum Setpoint {

        // Setpoints
        Idle(19.0, 0, 0.15, -0.7103000000000002),
        IntakeGround(0.5, 0, 2.175, 0.6),
        Speaker(19.0, 0, 0.15, -0.48640000000000017),
        Podium(40.0, 0, 0.15, 1.1652456),
        Wing(50.0, 0, 0.15, 1.1652456),
        Amp(116.0, 1.0, 0.6, 2),
        Trap(0.0, 0, 2.175, 0.25),
        Reset(45.0, 1.0, -0.25, -0.5),

        // Clearance Setpoints
        BumperOut(20.0, 0, 2.175, 4.0),
        BumperIn(16.0, 0, 0.15, -2.0),
        TrapOut(20.0, 0, 2.175, 0.75),
        TrapIn(16.0, 0, 0.15, 0.75),
        
        // Various Autonomous-only Angles
        Slot1(46.0, 0, 0.15, 1.1652456), // Vulture Human
        Slot2(43.0, 0, 0.15, 1.1652456), // Vulture Human
        Slot3(44.0, 0, 0.15, 1.1652456), // Do Not Use Amp
        Slot4(47.0, 0, 0.15, 1.1652456), // Do Not Use Amp
        Slot5(31.0, 0, 0.15, -0.7103000000000002), // Royal Flush
        Slot6(22.0, 0, 0.15, -0.7103000000000002), // Royal Flush
        Slot7(29.0, 0, 0.15, -0.7103000000000002), // Royal Flush
        Slot8(22.0, 0, 0.15, -0.7103000000000002), // Royal Flush
        Slot9(19.0, 0, 0.15, -0.48640000000000017), // All
        Slot10(28.0, 0, 0.15, -0.7103000000000002), // Royal Flush
        
        AutoPhodium(45, 0, 0.15, 1.1652456);
        
        private final double degrees;
        private final double rotations;
        private final double elevatorfeed;
        private final int slot;
        private final double armfeed;

        Setpoint(double arm, double armFeed, double elevator, double elevatorFeed) {
            this(arm, armFeed, elevator, elevatorFeed, 0);
        }

        Setpoint(double arm, double armFeed, double elevator, double elevatorFeed, int elevatorSlot) {
            this.degrees = arm;
            this.armfeed = armFeed;
            this.rotations = elevator;
            this.elevatorfeed = elevatorFeed;
            this.slot = elevatorSlot;
        }
        
        /**
         * Gets the desired rotations of the elevator for the given setpoint
         * @return Elevator rotations
         */
        public double getRotations() {
            return this.rotations;
        }
        
        /**
         * Gets the desired degrees of the arm for the given setpoint
         * @return Arm degrees
         */
        public double getDegrees() {
            return this.degrees;
        }
        
        /**
         * Gets the desired feed forward to the elevator for the given setpoint
         * @return Elevator feed forward in volts
         */
        public double getElevatorFeed() {
            return this.elevatorfeed;
        }

        /**
         * Gets the desired feed forward to the arm for the given setpoint
         * @return Arm feed forward in volts
         */
        public double getArmFeed() {
            return this.armfeed;
        }
        
        /**
         * Gets the desired Talon PID slot to use on the elevator for the given setpoint
         * @return Talon PID slot on elevator motors
         */
        public int getElevatorSlot() {
            return this.slot;
        }
    }

    public record ArmConfig(int topId, int bottomId, int encoderId) {};
    public record ElevatorConfig(int leftId, int rightId, int encoderId) {};
    public record Limits(double statorLimit, double supplyLimit, double forwardLimit, double reverseLimit) {};
    public record ClimberLimits(double statorLimit, double supplyLimit) {};
}
