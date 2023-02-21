// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Ports {
    public static class PistonTest{
        public static final int piston1ForwardPort = 5; //og 4
        public static final int piston1ReversePort = 4; //og 5
        public static final int piston2ForwardPort = 6;
        public static final int piston2ReversePort = 7;
        //public static final int cubeExtendButton = 1;
        

    }
    public class wrist{
        public static final int wristPort = 13; //8 
        // public static final int clawWheelsPort = 9; //9
    }
    public static class XboxControllerMap{
        public static class Button{
            public static final int X = XboxController.Button.kX.value;
            public static final int Y = XboxController.Button.kY.value;
            public static final int A = XboxController.Button.kA.value;
            public static final int B = XboxController.Button.kB.value;
        }
    }
}
