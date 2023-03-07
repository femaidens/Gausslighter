// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Ports.*;

public class ArmAngle extends SubsystemBase {
  private final CANSparkMax angleMotor;
  private final PIDController anglePIDController; // angle of lifting arm
  //private final SparkMaxAbsoluteEncoder angEncoder;
  private final DutyCycleEncoder angEncoder;
  private double armAngleVoltage;
  private double armAngle;

  public ArmAngle() {

    // motor instantiation
    angleMotor = new CANSparkMax(ArmPorts.ANG_MOTOR_PORT, MotorType.kBrushless);
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);

    // encoder instantiation
    angEncoder = new DutyCycleEncoder(ArmPorts.ANG_ENCODER_PORT);
    angEncoder.setPositionOffset(ArmConstants.ANGLE_OFFSET);
    // adjustedPosition = Math.abs(angEncoder.get());
    angEncoder.setDistancePerRotation(ArmConstants.ANGLE_FACTOR); // set units to degrees
    // angEncoder.setDutyCycleRange(0.0467, armAngleVoltage);

    // feedback controllers
    anglePIDController = new PIDController(
        ArmConstants.AnglePID.kP,
        ArmConstants.AnglePID.kI,
        ArmConstants.AnglePID.kD);
  }

  // arm angle pid not setting angle, change name later
  public void setAngle(double currentAngle, double goalAngle) {
    //INVERTED ANGLE MOTOR TO SPIN PROPERLY
    // positive speed = lower arm
    // negative speed = raise arm
    double armAngleVoltage = anglePIDController.calculate(currentAngle, goalAngle);
    angleMotor.setVoltage(armAngleVoltage);

    SmartDashboard.putNumber("Arm Voltage: ", armAngleVoltage);
  }

  public void downArm(){
    angleMotor.set(-0.3);
  }

  public void midAngle(){
    double currentPosition = 360-Math.abs(angEncoder.getDistance());
    if (currentPosition < ArmConstants.PositionConfig.midConeAngle - 2){
      angleMotor.set(0.3);
    }

    else if (currentPosition > ArmConstants.PositionConfig.midConeAngle + 2) {
      angleMotor.set(-0.2);
    }

    else{
      angleMotor.set(0);
    }
  }

  public double getArmAngle() {
    double currentAngle = -1*angEncoder.getDistance();
    return currentAngle;
  }

  public void stopAngleMotor() {
    angleMotor.set(0);
  }

  @Override
  public void periodic() {

    //SmartDashboard.putNumber("Arm Offset: ", angEncoder.getPositionOffset());
    SmartDashboard.putNumber("Arm Angle: ", 360-Math.abs(angEncoder.getDistance()));
    // SmartDashboard.putNumber("Arm Voltage: ", armAngleVoltage);
    SmartDashboard.putNumber("Unscaled position: ", angEncoder.get());
    SmartDashboard.putNumber("Abs Position: ", + angEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Zero Offset: ", + angEncoder.getPositionOffset());

    // SmartDashboard.putNumber("Scaled position: ", angEncoder.get()); // test after uncommenting the 360 unit conversion from rotations to degrees


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

// for cansparkmax encoder -> 1 rev = 4096 ticks

// default direction = clockwise