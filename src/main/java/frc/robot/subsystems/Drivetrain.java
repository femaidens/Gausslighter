// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Ports.*;
import frc.robot.subsystems.modules.RevSwerveModule;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

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

  // imu sensor
  private final ADIS16470_IMU gyro = new ADIS16470_IMU();

  // odometry class (tracks robot pose)
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });

  public Drivetrain() {
  }

  @Override
  public void periodic() {
    // updates periodically 
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });
    
    SmartDashboard.putNumber("Gyro Angle: ", getGyroValues());
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

  // sets desired swerve module states
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
  }

  // resets the odometry to the specified pose
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

  // returns currently-estimated pose of robot
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // returns heading in degrees (-180 to 180)
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  // returns turn rate (deg/s)
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  // public void testModules(){
  // }

  // x formation with wheels -> prevent movement
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public double getGyroValues(){
    return gyro.getAngle();
  }

  // zeros heading of the robot
  public void resetGyro(){
    gyro.reset();
  } 

  // resets drive encoders
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

}