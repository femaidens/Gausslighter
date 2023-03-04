// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public final class Ports {

    public static final class JoystickPorts {
        // public static final int ROTATION_JOY = 0;
        // public static final int LATERAL_JOY = 1;
        public static final int OPER_JOY = 0; // left side of laptop
    }

    public static final class XboxControllerMap {
        public static class Button {
            public static final int A = XboxController.Button.kA.value;
            public static final int B = XboxController.Button.kB.value;
            public static final int X = XboxController.Button.kX.value;
            public static final int Y = XboxController.Button.kY.value;
        }
    }

    public static final class ButtonPorts {
        // intake
        public static final int HP_BUTTON_PORT = 5;
        public static final int FLOOR_INTAKE_BUTTON_PORT = 15;
        public static final int RESET_INTAKE_BUTTON_PORT = 7;

        // score
        public static final int FLOOR_SCORE_BUTTON_PORT = 8; //8
    }
    
    public static final class DrivetrainPorts{
        // gyro port
        public static final int GYRO = 1;
        
        // big neos
        public static final int FRONT_LEFT_DRIVE = 4;
        public static final int FRONT_RIGHT_DRIVE = 3;
        public static final int REAR_LEFT_DRIVE = 6;
        public static final int REAR_RIGHT_DRIVE = 5;

        // small neos
        public static final int FRONT_LEFT_TURNING = 8;
        public static final int FRONT_RIGHT_TURNING = 7;
        public static final int REAR_LEFT_TURNING = 2;
        public static final int REAR_RIGHT_TURNING = 1;
    }

    public static final class IntakePorts {
        public static final int WRIST_MOTOR_PORT = 10;
      //  public static final int CLAW_WHEELS_PORT = 19;
        
        public static final int WRIST_ENCODER_PORT = 17;

        public static final int PISTON1_FORWARD_PORT = 0; // big piston
        public static final int PISTON1_REVERSE_PORT = 1; 
        public static final int PISTON2_FORWARD_PORT = 2; //small piston
        public static final int PISTON2_REVERSE_PORT = 3;
    }

    public static final class ArmPorts {
        public static final int ANG_MOTOR_PORT = 12;
        public static final int LEFT_EXTEND_MOTOR_PORT = 13;
        public static final int RIGHT_EXTEND_MOTOR_PORT = 9;

        public static final int ANG_ENCODER_PORT = 0; // change port number
        public static final int EXTEND_RETRACT_ENCODER_PORT = 8;

        public static final int TOP_SWITCH_PORT = 20; // change port num
        public static final int BOT_SWITCH_PORT = 21; // change port num
    }
}