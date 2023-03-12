// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports.*;
import frc.robot.Constants.*;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final DoubleSolenoid piston1;
  private final DoubleSolenoid piston2;
  private final CANSparkMax wristMotor;
  private final CANSparkMax clawMotor;
  private final PIDController wristAnglePID;
  // private final DutyCycleEncoder wristEncoder;
  private final AbsoluteEncoder wristEncoder;
  // private static int margin = 3; // margin for claw angle in terms of ticks,
  // can change later

  public Intake() {

    // piston instantiations
    piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakePorts.PISTON1_FORWARD_PORT,
        IntakePorts.PISTON1_REVERSE_PORT);
    piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakePorts.PISTON2_FORWARD_PORT,
        IntakePorts.PISTON2_REVERSE_PORT);

    // motor instantiations
    wristMotor = new CANSparkMax(IntakePorts.WRIST_MOTOR_PORT, MotorType.kBrushless);
    clawMotor = new CANSparkMax(IntakePorts.CLAW_MOTOR_PORT, MotorType.kBrushless);

    // encoder instantiations
    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    // wristEncoder = new DutyCycleEncoder(Ports.wrist.wristEncoderPort);

    // feedback controller
    wristAnglePID = new PIDController(IntakeConstants.PID.kP, 
        IntakeConstants.PID.kI, IntakeConstants.PID.kD);

    // encoder config
    wristEncoder.setPositionConversionFactor(360);
  }

  public void openClaw() { // retracting pistons
    piston1.set(Value.kReverse);
    piston2.set(Value.kReverse);
  }

  public void closeClawCube() { // extends pistons & clamps onto the gamepiece
    piston1.set(Value.kForward);
    piston2.set(Value.kForward);
    // System.out.println("both extended");
  }

  
  public void closeClawCone() { // extends pistons & clamps onto the gamepiece
    piston2.set(Value.kForward);
    // System.out.println("both extended");
  }

  // public void intakeGamePiece() {
  //   clawWheels.set(0.2); // subject to change, to negate
  // }

  // public void releaseGamePiece() { // backup plan???
  //   clawWheels.set(-0.2); // subject to change, to negate
  // }

  // public void stopClawWheels() {
  //   clawWheels.set(0);
  // }

  // PID
  public void setWristAnglePID(double goalAngle) {
    double goalTicks = goalAngle * IntakeConstants.tickFactor;
    // double goalTicks = 180 / (goalAngle * Math.PI) alternate method to convert a
    // goal angle into ticks

    // uses both (angle -> ticks) & (pos -> ticks) to calculate voltage
    double wristAngle = wristAnglePID.calculate(wristEncoder.getPosition(), goalTicks); 

    wristMotor.setVoltage(wristAngle);
  }

  // MANUAL

  public void setWristAngleManual(double input){
    wristMotor.set(input*0.5);
  }

  public boolean atWristAngle(double angle){
    double currentAngle = wristEncoder.getPosition()*360;
    //System.out.println("current angle: " + currentAngle);
    if (currentAngle <= angle + 2 && currentAngle > angle - 2) return true;
    return false; 
  }

  public void runIntakeMotor(){
    clawMotor.set(0.5);
  }

  public void stopWristMotor() {
    wristMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Angle", wristEncoder.getPosition());
    SmartDashboard.putBoolean("At wrist angle", atWristAngle(IntakeConstants.clawAngle));
  }
}