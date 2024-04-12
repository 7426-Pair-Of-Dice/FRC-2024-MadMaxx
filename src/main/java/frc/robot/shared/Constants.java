// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shared;

public final class Constants {
    /** A class of booleans used to enable or disable categories from sending to network tables. */
    public static final class Dashboard {
        /** Send all PID errors, typically on when tuning the PID of a subsystem. */
        public static final boolean kSendErrors = true;

        /** Send all PID reference points, typically sending the desired velocity or position of all PID loops. */
        public static final boolean kSendReferences = false;

        /** Sends all statistics on the current conditions of each subsystem, such as position, velocity, etc. */
        public static final boolean kSendStates = true;

        /** Sends all debug information, typically for testing things that do not fit into other categories. */
        public static final boolean kSendDebug = true;

        /** Sends all telemetry from the Phoenix 6 swerve template, this is only recommended when troubleshooting the drivetrain. */
        public static final boolean kSendSwerve = false;
    }

}
