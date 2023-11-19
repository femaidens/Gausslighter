// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.AngleConstants;
import frc.robot.Ports.ArmPorts;

public class armSim extends SubsystemBase {
  /** Creates a new armSim. */
  private final CANSparkMax angleMotor;
  private final SparkMaxAbsoluteEncoder angEncoder;
  private final PIDController anglePIDController;

  private final RelativeEncoder lateralEncoder; // extending and retraction arm
  // private final CANSparkMax rightLateralMotor;
  private final CANSparkMax leftLateralMotor;
  private final DigitalInput topSwitch;
  private final DigitalInput botSwitch;
  private final PIDController lateralPIDController;
  private final REVPhysicsSim simulator;
  public armSim() {
    angleMotor = new CANSparkMax(ArmPorts.ANG_MOTOR_PORT, MotorType.kBrushless);

    // motor configs
    angleMotor.setInverted(true);
    angleMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setSmartCurrentLimit(ArmConstants.ARM_ANGLE_MOTOR_CURRENT_LIMIT);
    anglePIDController = new PIDController(AngleConstants.kP, AngleConstants.kI, AngleConstants.kD);
    angEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // encoder configs
    angEncoder.setInverted(true);
    angEncoder.setPositionConversionFactor(360);

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

    lateralPIDController = new PIDController(AngleConstants.kP, AngleConstants.kI, AngleConstants.kD);
    // motor configs
    // rightLateralMotor.setIdleMode(IdleMode.kBrake);
    leftLateralMotor.setIdleMode(IdleMode.kBrake);

    leftLateralMotor.setInverted(false); // correct orientation
    // rightLateralMotor.setInverted(true); // makes retract negative

    // rightLateralMotor.setSmartCurrentLimit(ArmConstants.ARM_LATERAL_MOTOR_CURRENT_LIMIT);
    leftLateralMotor.setSmartCurrentLimit(ArmConstants.ARM_LATERAL_MOTOR_CURRENT_LIMIT);

    simulator = new REVPhysicsSim();
    simulator.addSparkMax()
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
