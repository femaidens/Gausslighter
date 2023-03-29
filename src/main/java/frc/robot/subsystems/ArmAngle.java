// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Ports.*;

public class ArmAngle extends SubsystemBase {
  private final CANSparkMax angleMotor;
  private final SparkMaxAbsoluteEncoder angEncoder;

  public ArmAngle() {
    // motor instantiation
    angleMotor = new CANSparkMax(ArmPorts.ANG_MOTOR_PORT, MotorType.kBrushless);

    // motor configs
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setSmartCurrentLimit(ArmConstants.ARM_ANGLE_MOTOR_CURRENT_LIMIT);

    // for cansparkmax encoder -> 1 rev = 4096 ticks
    // default direction = clockwise
    // encoder instantiation
    angEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // encoder configs
    angEncoder.setInverted(true);
    angEncoder.setPositionConversionFactor(360);
  
  }

  public void increaseArmAngle(){
    angleMotor.set(1);
    //System.out.print("increasing arm angle");
    System.out.println(angleMotor.getBusVoltage());
  }

  public void decreaseArmangle(){
    angleMotor.set(-1);
    //System.out.print("decreasing arm angle");
    System.out.println(angleMotor.getBusVoltage());

  }

  public void setAngle(double input){
    if (input == 0) angleMotor.set(0);
    angleMotor.set(-input);
    System.out.println(angleMotor.getBusVoltage());
  }

  public double getArmAngle() {
    double currentAngle = angEncoder.getPosition();
    return currentAngle;
  }

  public boolean atAngle(double angle){ //whether at angle w/ an offset of 2 degrees
    double currentAngle = angEncoder.getPosition();

    if (Math.abs(currentAngle - angle) < 1){ // change 1 back to 2 if margin is too small
      // new PrintCommand("at 36 degs");
      return true;
    }

    else {
      return false;
    }
  }

  public void stopAngleMotor() {
    angleMotor.set(0);
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
    //SmartDashboard.putNumber("arm angular speed", angleMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}