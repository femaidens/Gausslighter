// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  private static DoubleSolenoid piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.wrist.piston1ForwardPort, Ports.wrist.piston1ReversePort);
  private static DoubleSolenoid piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Ports.wrist.piston2ForwardPort, Ports.wrist.piston2ReversePort);
  private static CANSparkMax wrist = new CANSparkMax(Ports.wrist.wristMotorPort, MotorType.kBrushless);
  private static CANSparkMax clawWheels = new CANSparkMax(Ports.wrist.clawWheelsPort, MotorType.kBrushless);
  private static PIDController wristAngle = new PIDController(Constants.PIDConstants.Kp, Constants.PIDConstants.Ki, Constants.PIDConstants.Kd);
  private static DutyCycleEncoder wristEncoder = new DutyCycleEncoder(Ports.wrist.wristEncoderPort);
  private static int margin = 3; // margin for claw angle in terms of ticks, can change later

  public static void openClaw() { //retracting pistons
    piston1.set(Value.kReverse);
    piston2.set(Value.kReverse);
  }

  public static void closeClaw() { //extends pistons & clamps onto the gamepiece
    piston1.set(Value.kForward);
    piston2.set(Value.kForward);
    //System.out.println("both extended");
  }

  public static void intakeGamePiece() {
    clawWheels.set(0.2); //subject to change, to negate
  }

  public static void releaseGamePiece() { // backup plan???
    clawWheels.set(-0.2); //subject to change, to negate
  }

  //PID
  public void setWristAnglePID(double goalAngle){
    double setWristAnglePID = wristAngle.calculate(wristEncoder.getAbsolutePosition(), goalAngle);
    wrist.set(setWristAnglePID);
  }

  //MANUAL
  public void setWristAngleManual(double goalAngle) {
    double currentTicks = wristEncoder.getAbsolutePosition();
    double goalTicks = (goalAngle * Constants.CPR) / 360; // convert goal angle (degrees) into ticks, based on proportion of angle out of 360 and proportion of ticks out of 4096
  //double goalTicks = 180 / (goalAngle * Math.PI) alternate method to convert a goal angle into ticks

      if (currentTicks > goalTicks + margin || currentTicks < goalTicks - margin) {
        while(currentTicks < goalTicks - margin) {
          wrist.set(Constants.wristSpeed);
        }
        while(currentTicks > goalTicks + margin) {
          wrist.set(-Constants.wristSpeed);
        }
      }
      else {
        wrist.set(0);
      }

  }
  
  public void resetWristAngle() {
    while (wristEncoder.getAbsolutePosition() > margin) {
      wrist.set(-Constants.wristSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
