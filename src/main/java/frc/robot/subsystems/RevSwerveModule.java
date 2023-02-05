// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Constants.ModuleConstants.*;

public class RevSwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivePIDController;
  private final SparkMaxPIDController turningPIDController;
  private final SimpleMotorFeedforward driveFFController; // figure this out another day (optimize it?)

  private double chassisAngularOffset;
  private SwerveModuleState desiredState;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public RevSwerveModule(int drivingCANId, int turningCANId, double m_chassisAngularOffset) {
    driveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);

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

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public double calculateAngleOutput(SparkMaxPIDController controller, double setpoint, double measurement){
    return setpoint;
  }

  public double calculateVelocityOutput(SparkMaxPIDController controller, double setpoint, double measurement){
    return setpoint;
  }

  public void putValues(){
    // turning
    turningPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);

    // drive
    double velocity = driveEncoder.getVelocity();
    drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kPosition);

    SmartDashboard.putNumber("Velocity", velocity);
    SmartDashboard.putNumber("Target Velocity", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber("VkP", 0);
    SmartDashboard.putNumber("VkI", 0);
    SmartDashboard.putNumber("VkD", 0);

    // SmartDashboard.putNumber("Velocity", driveEncoder.getVelocity());
    // SmartDashboard.putNumber("Target Velocity", 0);
    // SmartDashboard.putNumber("VkP", 0);
    // SmartDashboard.putNumber("VkI", 0);
    // SmartDashboard.putNumber("VkD", 0);
    // SmartDashboard.putNumber("Angle", getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("Target Angle", 0);
    // SmartDashboard.putNumber("AkP", 0);
    // SmartDashboard.putNumber("AkI", 0);
    // SmartDashboard.putNumber("AkD", 0);
  }

  public void getValues(){

    // turning
    double turnOutput = calculateAngleOutput(turningPIDController, desiredState.angle.getDegrees(), getPosition().angle.getDegrees());
    turningMotor.set(turnOutput);
    turningPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);

    // drive
    double velocity = driveEncoder.getVelocity();
    double driveOutput = calculateVelocityOutput(drivePIDController, desiredState.speedMetersPerSecond, velocity);
    drivePIDController.setReference(driveOutput, ControlType.kVoltage);
    
    SmartDashboard.getNumber("Velocity", velocity);
    SmartDashboard.getNumber("Target Velocity", desiredState.speedMetersPerSecond);
    SmartDashboard.getNumber("VkP", 0);
    SmartDashboard.getNumber("VkI", 0);
    SmartDashboard.getNumber("VkD", 0);
    
    // SmartDashboard.getNumber("Velocity", velocity);
    // SmartDashboard.getNumber("Target Velocity", m_desiredState.speedMetersPerSecond);
    // SmartDashboard.getNumber("VkP", 0);
    // SmartDashboard.getNumber("Vk", 0);
    // SmartDashboard.getNumber("VkD", 0);
    // SmartDashboard.getNumber("Angle", getPosition().angle.getDegrees());
    // SmartDashboard.getNumber("Target Angle", m_desiredState.angle.getDegrees());
    // SmartDashboard.getNumber("AkP", 0);
    // SmartDashboard.getNumber("AkI", 0);
    // SmartDashboard.getNumber("AkD", 0);
  }
  //public void periodic() {
    // double turnOutput = turningPIDController.update(targetAngle.get(), turnMotor.getAngle());
    // turnMotor.set(turnOutput);

    // /** calculate output for drive motor */
    // double driveOutput = drivePIDController.update(targetVelocity.get(), driveMotor.getVelocity());
    // driveMotor.set(driveOutput);


  //}
  
}
