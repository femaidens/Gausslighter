// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Ports.*;

public class ArmAngle extends SubsystemBase {
  private final CANSparkMax angleMotor;
  private final PIDController anglePIDController; // angle of lifting arm
  private final DutyCycleEncoder angEncoder;

  public ArmAngle() {

    // motor instantiations
    angleMotor = new CANSparkMax(ArmPorts.ANG_MOTOR_PORT, MotorType.kBrushless);
    angleMotor.setInverted(true);

    // encoder instantiations
    angEncoder = new DutyCycleEncoder(ArmPorts.ANG_ENCODER_PORT);


    // feedback controllers
    anglePIDController = new PIDController(
        ArmConstants.AnglePID.kP,
        ArmConstants.AnglePID.kI,
        ArmConstants.AnglePID.kD);
  }

  // arm angle pid not setting angle, change name later
  public void setAngle(double goalAngle) {
    //INVERTED ANGLE MOTOR TO SPIN PROPERLY
    double adjustmentAngle = anglePIDController.calculate(angEncoder.getAbsolutePosition(), goalAngle);
    angleMotor.setVoltage(adjustmentAngle);
    System.out.println("angle pid: " + adjustmentAngle);
  }

  public void stopAngleMotor() {
    angleMotor.set(0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

// for cansparkmax encoder -> 1 rev = 4096 ticks

// default direction = clockwise