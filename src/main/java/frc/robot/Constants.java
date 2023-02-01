// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED = 4.8; // max speed meters per second
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.0);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot -> replace with known values
    
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // fl
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // fr
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // rl
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // rr

    // Angular offsets of the modules relative to the chassis in radians -> figure out
    public static final double FL_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FR_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double RL_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double RR_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    public static final boolean GYRO_REVERSED = false;
  }

  public static final class ModuleConstants {
    // 14T pinion gears
    public static final int DRIVE_MOTOR_PINION_TEETH = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER = 0.0762; // meters
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE)
        / DRIVE_MOTOR_REDUCTION;

    // drive encoder position factor
    public static final double DRIVE_ENCODER_PFACTOR = (WHEEL_DIAMETER * Math.PI)
        / DRIVE_MOTOR_REDUCTION; // meters
    // velocity factor
    public static final double DRIVE_ENCODER_VFACTOR = ((WHEEL_DIAMETER * Math.PI)
        / DRIVE_MOTOR_REDUCTION) / 60.0; // meters per second

    // turning encoder position factor
    public static final double TURNING_ENCODER_PFACTOR = (2 * Math.PI); // radians
    // velocity factor
    public static final double TURNING_ENCODER_VFACTOR = (2 * Math.PI) / 60.0; // radians per second

    // turning encoder position pid min input
    public static final double TURNING_ENCODER_PPID_MIN = 0; // radians
    // max input
    public static final double TURNING_ENCODER_PPID_MAX = TURNING_ENCODER_PFACTOR; // radians

    public static final double kDriveP = 0.03; // initially 0.04
    public static final double kDriveI = 0;
    public static final double kDriveD = 0.01;
    public static final double kDriveFF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double kDriveMinOutput = -1;
    public static final double kDriveMaxOutput = 1;

    public static final double kTurningP = 0.15; // initally 1
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.05;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 35; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 35; // amps
  }

  public static final class AutoConstants {
    public static final double AUTON_MAX_SPEED = 3; // max meters per second
    public static final double AUTON_MAX_ACC = 3; // max acc m/s^2
    public static final double AUTON_MAX_ANGULAR_SPEED = Math.PI; // max angular speed rad/s
    public static final double AUTON_MAX_ANGULAR_SPEED_SQUARED = Math.PI; // angular speed rad/s^2

    public static final double PXController = 1;
    public static final double PYController = 1;
    public static final double PThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        AUTON_MAX_ANGULAR_SPEED, AUTON_MAX_ANGULAR_SPEED_SQUARED);
  }

  public static final class NeoMotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  }
}
