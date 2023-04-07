// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.ArmConstants.AngleConstants;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Ports.*;

public class ArmAngle extends SubsystemBase {
  private final CANSparkMax angleMotor;
  private final SparkMaxAbsoluteEncoder angEncoder;
  private final PIDController anglePIDController;
  private boolean isManual = true;
  private double setpoint;
  private double armAngleVoltage = 0;

  public ArmAngle() {
    // motor instantiation
    angleMotor = new CANSparkMax(ArmPorts.ANG_MOTOR_PORT, MotorType.kBrushless);

    // motor configs
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setSmartCurrentLimit(ArmConstants.ARM_ANGLE_MOTOR_CURRENT_LIMIT);
    anglePIDController = new PIDController(AngleConstants.kP, AngleConstants.kI, AngleConstants.kD);
    // for cansparkmax encoder -> 1 rev = 4096 ticks
    // default direction = clockwise
    // encoder instantiation
    angEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // encoder configs
    angEncoder.setInverted(true);
    angEncoder.setPositionConversionFactor(360);

    setpoint = angEncoder.getPosition();
  
  }

  public double getArmAngle() {
    double currentAngle = angEncoder.getPosition();
    return currentAngle;
  }

  public boolean getIsManual(){
    return isManual;
  }

  public void setAutonArmAngle(double autonSetpoint){
    double autonArmAngleVoltage = anglePIDController.calculate(getArmAngle(), autonSetpoint);
    angleMotor.setVoltage(autonArmAngleVoltage);
    System.out.println(autonArmAngleVoltage);
  }

  public void increaseArmAngle(){
    angleMotor.set(1);
    //System.out.print("increasing arm angle");
    System.out.println(angleMotor.getBusVoltage());
  }

  public void decreaseArmAngle(){
    angleMotor.set(-1);
    //System.out.print("decreasing arm angle");
    System.out.println(angleMotor.getBusVoltage());
  }

  public void setAngle(double input){
    // if (input == 0) {
    //   setAngleVoltage();
    // }
    // else{
    // }
    //angleMotor.set(-input);
    // angleMotor.setVoltage(-6*input); 
    isManual = true;

    if (input < 0){ // neg input pushing up
      angleMotor.set(0.7);
      setpoint = angEncoder.getPosition();
    }
    else if (input > 0){ // pos input pushing down
      angleMotor.set(-0.7); // if input is positive, we go down
      setpoint = angEncoder.getPosition();
    }
    else {
      setManualAngleVoltage();
    }
  }

  public void setManualAngleVoltage(){
    armAngleVoltage = anglePIDController.calculate(angEncoder.getPosition(), setpoint);
    angleMotor.setVoltage(armAngleVoltage);
  }

  public void setAngleVoltage(){
    armAngleVoltage = anglePIDController.calculate(angEncoder.getPosition(), setpoint);
    angleMotor.setVoltage(armAngleVoltage);
  }

  public void setMidNodeAngle(){
    isManual = false;
    setpoint = 30;
  }

  public void setHighNodeAngle(){
    isManual = false;
    setpoint = 42.8;
  }

  public void setSingleArmAngle(){
    isManual = false;
    setpoint = 30;
 }

  public void setDoubleArmAngle(){
    isManual = false;
    setpoint = 42.8;
  }

  public void stopAngleMotor() {
    angleMotor.set(0);
  }

  public boolean atAngle(double angle){ //whether at angle w/ an offset of 2 degrees
    double currentAngle = angEncoder.getPosition();

    if (Math.abs(currentAngle - angle) < 2){ // change 1 back to 2 if margin is too small
      // new PrintCommand("at 36 degs");
      return true;
    }

    else {
      return false;
    }
  }

  @Override
  public void periodic() {

    // boolean boxes
    SmartDashboard.putBoolean("ARM DEFAULT ANG", atAngle(PositionConfig.defaultAngle));
    SmartDashboard.putBoolean("ARM LOW ANG", atAngle(PositionConfig.lowNodeAngle));
    SmartDashboard.putBoolean("ARM MID ANG", atAngle(PositionConfig.midNodeAngle));
    SmartDashboard.putBoolean("ARM HIGH ANG", atAngle(PositionConfig.highNodeAngle));
    SmartDashboard.putBoolean("ARM SINGLE ANG", atAngle(PositionConfig.hpSingleAngle));
    SmartDashboard.putBoolean("ARM DOUBLE ANG", atAngle(PositionConfig.hpDoubleAngle));

    // values
    SmartDashboard.putNumber("arm angle", angEncoder.getPosition());
    SmartDashboard.putNumber("desired arm angle", setpoint);

    //SmartDashboard.putNumber("arm angular speed", angleMotor.get());
    // armAngleVoltage = anglePIDController.calculate(angEncoder.getPosition(), setpoint);
    SmartDashboard.putNumber("arm angle v: ", armAngleVoltage);

    SmartDashboard.putNumber("kP", AngleConstants.kP);
    SmartDashboard.putNumber("kI", AngleConstants.kI);
    SmartDashboard.putNumber("kD", AngleConstants.kD);
    SmartDashboard.putNumber("applied v: ", angleMotor.getAppliedOutput());
    // SmartDashboard.putNumber("bus v: ", angleMotor.getBusVoltage());
    // SmartDashboard.putNumber("applied output: ", angleMotor.getAppliedOutput());



  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}