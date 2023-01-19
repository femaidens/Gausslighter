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
        public static final int FRONT_LEFT_DRIVE = 4;
        public static final int REAR_LEFT_DRIVE = 10;
        public static final int FRONT_RIGHT_DRIVE = 15;
        public static final int REAR_RIGHT_DRIVE = 17;

        // small neos
        public static final int FRONT_LEFT_TURNING = 1;
        public static final int REAR_LEFT_TURNING = 15;
        public static final int FRONT_RIGHT_TURNING = 14;
        public static final int REAR_RIGHT_TURNING = 16;
    }
}
