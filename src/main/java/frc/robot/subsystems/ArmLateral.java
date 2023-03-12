// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Ports.ArmPorts;

public class ArmLateral extends SubsystemBase {
  private final RelativeEncoder extendRetractEncoder; // extending and retraction arm
  private final CANSparkMax rightExtendMotor;
  private final CANSparkMax leftExtendMotor;
  // private final PIDController extendPIDController;
  private final SimpleMotorFeedforward armExtensionFF; // -> use this somewhere!
  private final DigitalInput topSwitch;
  private final DigitalInput botSwitch;

  public ArmLateral() {

    // motor instantiations
    rightExtendMotor = new CANSparkMax(ArmPorts.RIGHT_EXTEND_MOTOR_PORT, MotorType.kBrushless);
    leftExtendMotor = new CANSparkMax(ArmPorts.LEFT_EXTEND_MOTOR_PORT, MotorType.kBrushless);

    // encoder instantiations
    extendRetractEncoder = leftExtendMotor.getEncoder(); // subject to change

    // feedback controllers

    // extendPIDController = new PIDController(
    //     ArmConstants.ExtendPID.kP,
    //     ArmConstants.ExtendPID.kI,
    //     ArmConstants.ExtendPID.kD);

    // feedforward controllers
    armExtensionFF = new SimpleMotorFeedforward(
        ArmConstants.FeedForward.kS,
        ArmConstants.FeedForward.kV, 
        ArmConstants.FeedForward.kA);

    // limit switches
    topSwitch = new DigitalInput(ArmPorts.TOP_SWITCH_PORT);
    botSwitch = new DigitalInput(ArmPorts.BOT_SWITCH_PORT);

    // motor config
    leftExtendMotor.setInverted(true);
  }

  // public void extendPosition(double position) {
  //   double extendArmPID = extendPIDController.calculate(extendRetractEncoder.getPosition(), position);
  //   // double extendArmFF = velocityFF.calculate(vVelocity); just testing pid for
  //   // now
  //   leftExtendMotor.setVoltage(extendArmPID); // velocity PID/FF returns voltage

  //   if (extendArmPID < 0) { // left is extending 
  //     if (topSwitch.get()) { // hit top limit switch (extending)
  //       leftExtendMotor.set(0);
  //       rightExtendMotor.set(0);
  //     } 
      
  //     else {
  //       leftExtendMotor.setVoltage(extendArmPID); // velocity PID/FF returns voltage
  //       rightExtendMotor.setVoltage(extendArmPID);
  //     }
  //   } 
    
  //   else {
  //     if (botSwitch.get()) { // hit bot limit switch (retracting)
  //       leftExtendMotor.set(0);
  //       rightExtendMotor.set(0);
  //     } 

  //     else {
  //       leftExtendMotor.setVoltage(extendArmPID); // velocity PID/FF returns voltage
  //       rightExtendMotor.setVoltage(extendArmPID);
  //     }
  //   }

  //   System.out.println("extend arm pid: " + extendArmPID);
  //   // rightExtendMotor.setVoltage(extendArmPID); //negate one side
  // }

  // public void setLength(double input){
  //   if (input == 0) stopExtensionMotors();
  //   // if (topSwitch.get() || botSwitch.get()) { //hit limit switch
  //   //   stopExtensionMotors();
  //   // }
  //   // else{
  //     leftExtendMotor.set(input*0.7); //right direction?
  //     rightExtendMotor.set(input*0.7);
  //   // }
  // }
  public void retractArm(double input){
    if (input == 0) stopExtensionMotors();
    // if (topSwitch.get() || botSwitch.get()) { //hit limit switch
    //   stopExtensionMotors();
    // }
    // else{
      leftExtendMotor.set(-input*0.7); //right direction?
      rightExtendMotor.set(-input*0.7);
    // }
  }
public void extendArm(double input){
  if (input == 0) stopExtensionMotors();
  // if (topSwitch.get() || botSwitch.get()) { //hit limit switch
  //   stopExtensionMotors();
  // }
  // else{
    leftExtendMotor.set(-input*0.7); //right direction?
    rightExtendMotor.set(-input*0.7);
  // }
}
  public boolean atLength(double length){
    double currentLength = extendRetractEncoder.getPosition(); //might have to find a scale factor
    if (currentLength <= length + 2 && currentLength > length - 2) return true;
    return false;
  }

  public void getTicks() {
    // check to see if ticks are inverted (since motor is inverted); if so invert
    // encoder values in constructor
    leftExtendMotor.set(-0.15); // negative = extend
    System.out.println("current ticks:" + extendRetractEncoder.getPosition());
  }

  // find distance/revolution, return double after confirmation -> works
  public void getPositionFactor(double ticks) {
    double currentTicks = extendRetractEncoder.getPosition();

    while (currentTicks < ticks) {
      leftExtendMotor.set(-0.05);
      System.out.println("current ticks: " + currentTicks);
    }
  }

  // public void extendDistance(double distance) {
  //   double currRotations = extendRetractEncoder.getPosition();
  //   // set the parameter equal to method after test
  //   // if method works, use getPostionFactor in constructor
  //   /*
  //    * while (currTicks < Constants.CPR) {
  //    * System.out.println("current ticks: " + currTicks);
  //    * leftExtendMotor.set(-0.05);
  //    * }
  //    */
  //   // if(currRotations != distance) {
  //   while (currRotations < distance) { // used to add margin
  //     System.out.println("current rotations: " + currRotations);
  //     leftExtendMotor.set(-0.3);
  //   }
  //   // while(currRotations > distance + extendMargin) {
  //   // System.out.println("current rotations: " + currRotations);
  //   // leftExtendMotor.set(0.15);
  //   // }
  //   // }
  // }

  public void stopExtensionMotors() {
    leftExtendMotor.set(0);
    rightExtendMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("At Length: ", atLength(ArmConstants.PositionConfig.midLength)); //random length
    SmartDashboard.putNumber("Arm Position: ", extendRetractEncoder.getPosition());
    SmartDashboard.putNumber("Arm Lateral Speed: ", leftExtendMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
