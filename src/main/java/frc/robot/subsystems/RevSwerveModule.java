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
import frc.robot.Constants.ModuleConstants;

public class RevSwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkMaxPIDController drivePIDController;
  // private final SimpleMotorFeedforward driveFFController; // figure this out another day (optimize it?)
  private final SparkMaxPIDController turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public RevSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    driveMotor = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
    drivePIDController = driveMotor.getPIDController();
    turningPIDController = turningMotor.getPIDController();
    drivePIDController.setFeedbackDevice(driveEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_PFACTOR);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VFACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_PFACTOR);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.TURNING_ENCODER_VFACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.TURNING_ENCODER_PPID_MIN);
    turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.TURNING_ENCODER_PPID_MAX);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivePIDController.setP(ModuleConstants.kDriveP);
    drivePIDController.setI(ModuleConstants.kDriveI);
    drivePIDController.setD(ModuleConstants.kDriveD);
    drivePIDController.setFF(ModuleConstants.kDriveFF);
    drivePIDController.setOutputRange(ModuleConstants.kDriveMinOutput,
        ModuleConstants.kDriveMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPIDController.setP(ModuleConstants.kTurningP);
    turningPIDController.setI(ModuleConstants.kTurningI);
    turningPIDController.setD(ModuleConstants.kTurningD);
    turningPIDController.setFF(ModuleConstants.kTurningFF);
    turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    driveMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    // driveMotor.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    // turningMotor.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    driveMotor.burnFlash();
    turningMotor.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }

  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public void putValues(){
    // turning
    turningPIDController.setReference(m_desiredState.angle.getRadians(), ControlType.kPosition);

    // drive
    double velocity = driveEncoder.getVelocity();
    drivePIDController.setReference(m_desiredState.speedMetersPerSecond, ControlType.kPosition);

    SmartDashboard.putNumber("drive P", ModuleConstants.kDriveP);
    SmartDashboard.putNumber("drive I", ModuleConstants.kDriveI);
    SmartDashboard.putNumber("drive D", ModuleConstants.kDriveD);
    //displaying pid values for turning motors
    SmartDashboard.putNumber("turn P", ModuleConstants.kTurningP);
    SmartDashboard.putNumber("turn I", ModuleConstants.kTurningI);
    SmartDashboard.putNumber("turn D", ModuleConstants.kTurningD);

    SmartDashboard.putNumber("Velocity", velocity);
    SmartDashboard.putNumber("Target Velocity", m_desiredState.speedMetersPerSecond);

    SmartDashboard.putNumber("Angle", getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Target Angle", m_desiredState.angle.getDegrees());
  }

  public void getValues(){

    // turning
    double turnOutput = turningPIDController.calculate(m_desiredState.angle.getDegrees(), getPosition().angle);
    turningMotor.set(turnOutput);
    turningPIDController.setReference(m_desiredState.angle.getRadians(), ControlType.kPosition);

    // drive
    double velocity = driveEncoder.getVelocity();
    double driveOutput = drivePIDController.calculate(m_desiredState.speedMetersPerSecond, velocity);
    drivePIDController.setReference(driveOutput, ControlType.kVoltage);
    
    //displaying pid values for drive motors
    SmartDashboard.getNumber("drive P", ModuleConstants.kDriveP);
    SmartDashboard.getNumber("drive I", ModuleConstants.kDriveI);
    SmartDashboard.getNumber("drive D", ModuleConstants.kDriveD);
    //displaying pid values for turning motors
    SmartDashboard.getNumber("turn P", ModuleConstants.kTurningP);
    SmartDashboard.getNumber("turn I", ModuleConstants.kTurningI);
    SmartDashboard.getNumber("turn D", ModuleConstants.kTurningD);

    SmartDashboard.getNumber("Velocity", velocity);
    SmartDashboard.getNumber("Target Velocity", m_desiredState.speedMetersPerSecond);

    SmartDashboard.getNumber("Angle", getPosition().angle.getDegrees());
    SmartDashboard.getNumber("Target Angle", m_desiredState.angle.getDegrees());
  }
  //public void periodic() {
    // double turnOutput = turningPIDController.update(targetAngle.get(), turnMotor.getAngle());
    // turnMotor.set(turnOutput);

    // /** calculate output for drive motor */
    // double driveOutput = drivePIDController.update(targetVelocity.get(), driveMotor.getVelocity());
    // driveMotor.set(driveOutput);


  //}
  
}
