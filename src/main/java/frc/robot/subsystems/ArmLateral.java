// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.AngleConstants;
import frc.robot.Constants.ArmConstants.LateralConstants;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Ports.ArmPorts;

public class ArmLateral extends SubsystemBase {
  private final RelativeEncoder lateralEncoder; // extending and retraction arm
  // private final CANSparkMax rightLateralMotor;
  private final CANSparkMax leftLateralMotor;
  private final DigitalInput topSwitch;
  private final DigitalInput botSwitch;
  private final PIDController ArmPIDController;
  private double currentLength = 0; // always true before match, arm = fully retracted
  public static boolean isOverridden = false;

  public ArmLateral() {

    // motor instantiations
    // rightLateralMotor = new CANSparkMax(ArmPorts.RIGHT_EXTEND_MOTOR_PORT, MotorType.kBrushless);
    leftLateralMotor = new CANSparkMax(ArmPorts.LEFT_EXTEND_MOTOR_PORT, MotorType.kBrushless);

    // encoder instantiations
    lateralEncoder = leftLateralMotor.getEncoder(); // subject to change

    // encoder config - check/test for pfactor
    // lateralEncoder.setPositionConversionFactor(LateralConstants.LATERAL_PFACTOR); //divide by revs for fully extend

    // limit switches
    topSwitch = new DigitalInput(ArmPorts.TOP_SWITCH_PORT);
    botSwitch = new DigitalInput(ArmPorts.BOT_SWITCH_PORT);

    ArmPIDController = new PIDController(AngleConstants.kP, AngleConstants.kI, AngleConstants.kD);
    // motor configs
    // rightLateralMotor.setIdleMode(IdleMode.kBrake);
    leftLateralMotor.setIdleMode(IdleMode.kBrake);

    leftLateralMotor.setInverted(false); // correct orientation
    // rightLateralMotor.setInverted(true); // makes retract negative

    // rightLateralMotor.setSmartCurrentLimit(ArmConstants.ARM_LATERAL_MOTOR_CURRENT_LIMIT);
    leftLateralMotor.setSmartCurrentLimit(ArmConstants.ARM_LATERAL_MOTOR_CURRENT_LIMIT);
  }

  public void retractArm(boolean isOverriden){
    if (isOverriden) {
      leftLateralMotor.set(-0.9);
    } 

    else {
      if (!botSwitch.get()) { //hit limit switch (true = not hit; false = hit)
        System.out.println(" bottom limit activated! \n");
        stopExtensionMotors();
        lateralEncoder.setPosition(0);
        currentLength = lateralEncoder.getPosition();
        return;
      }
  
      else {
        System.out.println("retracting");
        leftLateralMotor.set(-0.9);
        // rightLateralMotor.set(-0.4);
        // currentLength = LateralConstants.LATERAL_LENGTH + lateralEncoder.getPosition(); 
        // change back to minus if increases moving backwards
      }
    }
  }

  public void extendArm(boolean isOverriden) {
    lateralEncoder.setPosition(currentLength);
    //System.out.println("rotations: " +lateralEncoder.getPosition());
      if (isOverriden) {
        leftLateralMotor.set(0.9);
      } 
      
      else {
        if (!topSwitch.get()) { //hit limit switch
          System.out.println("top limit activated! p\n");
          stopExtensionMotors();
          // lateralEncoder.getPosition();
          // lateralEncoder.setPosition(0);
          return; 
        }

        else {
          leftLateralMotor.set(0.9);
        // rightLateralMotor.set(0.4);
        // currentLength = lateralEncoder.getPosition();
        }
       } 
  }

  public boolean hitTopSwitch(){
    return !topSwitch.get();
  }

  public void isOverridden() {
    isOverridden = true;
  }

  public void isNotOverridden(){
    isOverridden = false;
  }
  
  public void setAutonArmLength(double autonSetpoint){
    double autonArmLateralVoltage = ArmPIDController.calculate(currentLength, autonSetpoint);
    leftLateralMotor.setVoltage(autonArmLateralVoltage);
  }

  public boolean atLength(double length){
    if (Math.abs(currentLength - length) < 1) return true;
    return false;
  }

  public void stopExtensionMotors() {
    leftLateralMotor.set(0);
    // rightLateralMotor.set(0);
  }

  @Override
  public void periodic() {
    
    // boolean boxes
    // SmartDashboard.putBoolean("@ low length", atLength(ArmConstants.PositionConfig.lowLength)); 
    // SmartDashboard.putBoolean("@ mid length", atLength(ArmConstants.PositionConfig.midLength)); <--
    // SmartDashboard.putBoolean("@ high length", atLength(ArmConstants.PositionConfig.highLength)); <--
    // SmartDashboard.putBoolean("@ hp length", atLength(ArmConstants.PositionConfig.doubleHPLength));
    SmartDashboard.putBoolean("@ default length", atLength(ArmConstants.PositionConfig.defaultLength));

    
    // arm lateral values
    //SmartDashboard.putNumber("get current length", currentLength);
    // SmartDashboard.putNumber("get lateral length ", lateralEncoder.getPosition()); <--
    // SmartDashboard.putNumber("get conversion factor ", lateralEncoder.getPositionConversionFactor());


    //SmartDashboard.putNumber("arm lateral speed", leftLateralMotor.get());
    //SmartDashboard.putNumber("lateral pfactor", lateralEncoder.getPositionConversionFactor());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}