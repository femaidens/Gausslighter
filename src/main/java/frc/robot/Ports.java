// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Ports {

    public static final class JoystickPorts {
        public static final int ROTATION_JOY = 0; // port closest to us
        public static final int LATERAL_JOY = 1;
        public static final int OPER_JOY = 2; // left side of laptop
    }
    
    public static final class DrivetrainPorts{
        // big neos
        public static final int kFrontLeftDrivingCanId = 4;
        public static final int kRearLeftDrivingCanId = 10;
        // public static final int kFrontRightDrivingCanId = 15;
        // public static final int kRearRightDrivingCanId = 17;

        // small neos
        public static final int kFrontLeftTurningCanId = 1;
        public static final int kRearLeftTurningCanId = 15;
        // public static final int kFrontRightTurningCanId = 14;
        // public static final int kRearRightTurningCanId = 16;
    }
}
