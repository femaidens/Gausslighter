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
  private double adjustmentAngle;

  public ArmAngle() {

    // motor instantiation
    angleMotor = new CANSparkMax(ArmPorts.ANG_MOTOR_PORT, MotorType.kBrushless);
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kCoast);

    // encoder instantiation
    angEncoder = new DutyCycleEncoder(ArmPorts.ANG_ENCODER_PORT);
    angEncoder.setPositionOffset(ArmConstants.ANGLE_OFFSET);
  angEncoder.setDistancePerRotation(360); // set units to degrees
    // angEncoder.setDutyCycleRange(0.0467, adjustmentAngle);
    // angEncoder.setDistancePerRotation(ArmConstants.ANGLE_FACTOR);
    
    // feedback controllers
    anglePIDController = new PIDController(
        ArmConstants.AnglePID.kP,
        ArmConstants.AnglePID.kI,
        ArmConstants.AnglePID.kD);
  }

  // arm angle pid not setting angle, change name later
  public void setAngle(double goalAngle) {
    //INVERTED ANGLE MOTOR TO SPIN PROPERLY
    // positive speed = lower arm
    // negative speed = raise arm
    double adjustmentAngle = anglePIDController.calculate(angEncoder.getDistance(), goalAngle);
    angleMotor.setVoltage(adjustmentAngle);


    System.out.println("Arm Voltage: " + adjustmentAngle);
  }

  public double getArmAngle() {
    double currentAngle = angEncoder.getDistance();
    return currentAngle;
  }

  public void stopAngleMotor() {
    angleMotor.set(0);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Arm Offset: ", angEncoder.getPositionOffset());
    SmartDashboard.putNumber("Arm Angle: ", angEncoder.getDistance());
    SmartDashboard.putNumber("Arm Voltage: ", adjustmentAngle);
    SmartDashboard.putNumber("Unscaled position: ", angEncoder.get());
    SmartDashboard.putNumber("Abs Position: ", + angEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("zero offset: ", + angEncoder.getPositionOffset());

    // SmartDashboard.putNumber("Scaled position: ", angEncoder.get()); // test after uncommenting the 360 unit conversion from rotations to degrees


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

// for cansparkmax encoder -> 1 rev = 4096 ticks

// default direction = clockwise