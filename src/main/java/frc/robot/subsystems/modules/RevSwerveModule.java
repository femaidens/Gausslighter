// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.modules;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.*;
import frc.robot.Constants.ModuleConstants.*;

public class RevSwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivePIDController;
  private final SparkMaxPIDController turningPIDController;
  private final SimpleMotorFeedforward driveFFController;

  private double chassisAngularOffset;
  
  private SwerveModuleState desiredState;

  public RevSwerveModule(int drivingCANId, int turningCANId, double m_chassisAngularOffset) {

    driveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    // encoder setup
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePIDController = driveMotor.getPIDController();
    turningPIDController = turningMotor.getPIDController();

    drivePIDController.setFeedbackDevice(driveEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);
    driveFFController = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);

    // convert encoder values to units
    driveEncoder.setPositionConversionFactor(Drive.DRIVE_ENCODER_PFACTOR); // m
    driveEncoder.setVelocityConversionFactor(Drive.DRIVE_ENCODER_VFACTOR); // m/s

    turningEncoder.setPositionConversionFactor(Turning.ENCODER_PFACTOR); // rad
    turningEncoder.setVelocityConversionFactor(Turning.ENCODER_VFACTOR); // rad/s

    // output shaft rotates in the opp dir. of the steering motor
    turningEncoder.setInverted(Turning.ENCODER_INVERTED);

    // pid wrapping -> allows controller to go through 0 to reach setpoint
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(Turning.ENCODER_PPID_MIN);
    turningPIDController.setPositionPIDWrappingMaxInput(Turning.ENCODER_PPID_MAX);

    // drive PID
    drivePIDController.setP(Drive.kP);
    drivePIDController.setI(Drive.kI);
    drivePIDController.setD(Drive.kD);
    drivePIDController.setFF(Drive.kFF);
    drivePIDController.setOutputRange(Drive.kMinOutput,
        Drive.kMaxOutput);

    // turning PID
    turningPIDController.setP(Turning.kP);
    turningPIDController.setI(Turning.kI);
    turningPIDController.setD(Turning.kD);
    turningPIDController.setFF(Turning.kFF);
    turningPIDController.setOutputRange(Turning.kMinOutput,
        Turning.kMaxOutput);

    driveMotor.setIdleMode(ModuleConstants.kDriveMotorIdleMode);
    turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    turningMotor.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

    // saves sparkmax configs
    driveMotor.burnFlash();
    turningMotor.burnFlash();

    chassisAngularOffset = m_chassisAngularOffset;
    desiredState = new SwerveModuleState(0.0, new Rotation2d());
    driveEncoder.setPosition(0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */

  public SwerveModuleState getState() {
    // gets position relative to chassis (minus offset)
    return new SwerveModuleState(
        driveEncoder.getVelocity(),
        Rotation2d.fromRadians(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */

  public SwerveModulePosition getPosition() {
    // gets position relative to chassis (minus offset)
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRadians(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState m_desiredState) {
    // apply chassis angular offset to desired state
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = m_desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = m_desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // optimize to avoid spinning > 90 deg
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));
    
    // setpoints for drive & turning sparkmaxes
    double driveFF = driveFFController.calculate(optimizedDesiredState.speedMetersPerSecond);
    
    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, driveFF);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    optimizedDesiredState = m_desiredState;
  }

  public void periodic(){
    double velocity = driveEncoder.getVelocity();

    // smartdashboard values
    SmartDashboard.putNumber("Current Velocity: ", velocity);
    SmartDashboard.putNumber("Target Velocity: ", desiredState.speedMetersPerSecond);

    SmartDashboard.putNumber("Current Angle: ", getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Target Angle; ", desiredState.angle.getDegrees());
  }
}