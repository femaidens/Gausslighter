// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class TestSolenoid extends SubsystemBase {
  /** Creates a new TestSolenoid. */
  private static DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.PistonTest.piston1ForwardPort, Ports.PistonTest.piston1ReversePort);
  private static DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.PistonTest.piston2ForwardPort, Ports.PistonTest.piston2ReversePort);
  //private static CANSparkMax wrist = new CANSparkMax(Ports.wrist.wristPort, MotorType.kBrushless);
  //private static CANSparkMax clawWheels = new CANSparkMax(Ports.wrist.clawWheelsPort, MotorType.kBrushless);

  public TestSolenoid() {}

  public static void extendCone(){ // for cone extend piston1
    piston1.set(Value.kForward);
    //System.out.println("piston1 extended");
  }
  //public static void retractCone(){
  //  piston1.set(Value.kReverse);
  //}
  public static void extendCube(){
    piston1.set(Value.kForward);
    piston2.set(Value.kForward);
    //System.out.println("both extended");

  }
  public static void retractCone(){
    piston1.set(Value.kReverse);
    piston2.set(Value.kReverse);
    //System.out.println("both retracted");

  }
  public static void retractCube(){
    piston1.set(Value.kReverse);
    System.out.println("one retracted");

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}