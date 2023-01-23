// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Ports.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Create RevSwerveModules
  private final RevSwerveModule frontLeft = new RevSwerveModule(
      DrivetrainPorts.FRONT_LEFT_DRIVE,
      DrivetrainPorts.FRONT_LEFT_TURNING,
      DriveConstants.FL_CHASSIS_ANGULAR_OFFSET);

  private final RevSwerveModule frontRight = new RevSwerveModule(
      DrivetrainPorts.FRONT_RIGHT_DRIVE,
      DrivetrainPorts.FRONT_RIGHT_TURNING,
      DriveConstants.FR_CHASSIS_ANGULAR_OFFSET);

  private final RevSwerveModule rearLeft = new RevSwerveModule(
      DrivetrainPorts.REAR_LEFT_DRIVE,
      DrivetrainPorts.REAR_LEFT_TURNING,
      DriveConstants.RL_CHASSIS_ANGULAR_OFFSET);

  private final RevSwerveModule rearRight = new RevSwerveModule(
      DrivetrainPorts.REAR_RIGHT_DRIVE,
      DrivetrainPorts.REAR_RIGHT_TURNING,
      DriveConstants.RR_CHASSIS_ANGULAR_OFFSET);

  // // instantiating turning motors
  // private final CANSparkMax frontLeftTurning = new CANSparkMax(DrivetrainPorts.FRONT_LEFT_TURNING, MotorType.kBrushless);
  // private final CANSparkMax frontRightTurning = new CANSparkMax(DrivetrainPorts.FRONT_RIGHT_TURNING, MotorType.kBrushless);
  // private final CANSparkMax rearLeftTurning = new CANSparkMax(DrivetrainPorts.REAR_LEFT_TURNING, MotorType.kBrushless);
  // private final CANSparkMax rearRightTurning = new CANSparkMax(DrivetrainPorts.REAR_RIGHT_TURNING, MotorType.kBrushless);

  // instantiating turning absolute encoders
  // private final SparkMaxAbsoluteEncoder frontLeftTurningEncoder = frontLeftTurning.getAbsoluteEncoder(Type.kDutyCycle);
  // // private final DutyCycleEncoder frontLeftTurningEncoder = new DutyCycleEncoder(DrivetrainPorts.FRONT_LEFT_TURNING_ENCODER);
  // private final SparkMaxAbsoluteEncoder frontRightTurningEncoder = frontRightTurning.getAbsoluteEncoder(Type.kDutyCycle);
  // private final SparkMaxAbsoluteEncoder rearLeftTurningEncoder = rearLeftTurning.getAbsoluteEncoder(Type.kDutyCycle);
  // private final SparkMaxAbsoluteEncoder rearRightTurningEncoder = rearRightTurning.getAbsoluteEncoder(Type.kDutyCycle);

  // The gyro sensor
  private final AnalogGyro gyro = new AnalogGyro(DrivetrainPorts.GYRO);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
  }

  // public void stopTurning(){
  //   rearRightTurning.set(0);
  // }
  
  // public void getTurningPosition(){
  //   System.out.println("current position: " + frontLeftTurningEncoder.getPosition());
  // }

  // public void turnMotorForward(){
  //   // FL counter clockwise, increasing position value
  //   // FR counter clockwise, incresaing position
  //   // RR counter, decreasing
  //   rearRightTurning.set(0.03); 
  //   System.out.println("front left position: " + rearRightTurningEncoder.getPosition() + "\n zero offset: " + rearRightTurningEncoder.getZeroOffset());
  // }

  // public void turnMotorBackward(){
  //   // FL clockwise, decreasing position value
  //   // FR clockwise, decreasing position value
  //   // RR clockwise, increasing
  //   rearRightTurning.set(-0.03);
  //   System.out.println("front left position: " + rearRightTurningEncoder.getPosition() + "\n zero offset: " + rearRightTurningEncoder.getZeroOffset());
  // }

  // public void testTurning(){
  //   System.out.println("front left initial position: " + frontLeftTurningEncoder.getPosition());
  //   while(frontLeftTurningEncoder.getPosition() > 0.1){
  //     if(frontLeftTurningEncoder.getPosition() > 0.1){
  //       frontLeftTurning.set(-0.02);
  //     }
      
  //     else{
  //       frontLeftTurning.set(0.0);
  //     }
  //     // frontRightTurning.set(0.05);
  //     // rearLeftTurning.set(0.05);
  //     // frontRightTurning.set(0.05);
  //     System.out.println("front left position: " + frontLeftTurningEncoder.getPosition() + "\n zero offset: " + frontLeftTurningEncoder.getZeroOffset());
  //     System.out.println();
  //     // System.out.println("front right position: " + frontRightTurningEncoder.getPosition()  + "\n zero offset: " + frontRightTurningEncoder.getZeroOffset());
  //     // System.out.println();
  //     // System.out.println("rear left position: " + rearLeftTurningEncoder.getPosition() + "\n zero offset: " + rearLeftTurningEncoder.getZeroOffset());
  //     // System.out.println();
  //     // System.out.println("rear right position: " + rearRightTurningEncoder.getPosition() + "\n zero offset: " + rearRightTurningEncoder.getZeroOffset());
  //     // System.out.println();
  //   }
  // }
  
  // @Override
  public void periodic() {
    // // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Adjust input based on max speed
    xSpeed *= DriveConstants.MAX_SPEED;
    ySpeed *= DriveConstants.MAX_SPEED;
    rot *= DriveConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyro.getAngle()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void testModules(){

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  
}
