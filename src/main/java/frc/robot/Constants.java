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
  
  public static final double CPR = 4096;
  public static final double PPR = 1024;
  public static final double WHEEL_DIAMETER = 0.0762; // meters

  public static class LEDConstants {
    //public static final int LED_DATA_LENGTH = 1; //change as necessary
    public static final int LED_PIN_LENGTH = 177;
  }
  public static class ArmConstants {

    // public static final double EXTEND_MARGIN = 0.1;
    // public static final double EXTEND_FACTOR = (WHEEL_DIAMETER * Math.PI) / CPR; // test for distance per rotation
    
    // public static final double ANGLE_FACTOR = 360; // 360 deg/rev
    // public static final double ANGLE_OFFSET = 0.046808;
    public static final int ARM_ANGLE_MOTOR_CURRENT_LIMIT = 30; // amps
    public static final int ARM_LATERAL_MOTOR_CURRENT_LIMIT = 30; // amps

    public static class AngleConstants{
      public static final double kP = 3;
      public static final double kI = 0;
      public static final double kD = 0.05;


    }
    public static class LateralConstants{
      public static final double LATERAL_LENGTH = 26.375; // in inches distance to extend
      public static final double LATERAL_ROTATIONS = 21.015; // test
      public static final double  LATERAL_PFACTOR = LATERAL_LENGTH/LATERAL_ROTATIONS; //divide this by # of revs for fully extend
    }

    public static class PositionConfig { // subject to change
      // arm angles (for both intakes)
      public static final double defaultAngle = 63.9;
      public static final double lowNodeAngle = 1.986;
      public static final double midNodeAngle = 30; 
      public static final double highNodeAngle = 30;
      public static final double hpSingleAngle = 30; //tbd
      public static final double hpDoubleAngle = 40;
      // public static final double autonScoreAngle = 40;

      // starting: 47 || high: 73 || mid: 53.5 || low: 44 || double hp station: 60.069
      // find unit conversion units, current: inches
      // desired extension distance -> change after raised crossbar
      public static final double defaultLength = 0;
      public static final double lowLength = defaultLength;
      public static final double midLength = 24.26;
      public static final double highLength = 46.3; 
      public static final double doubleHPLength = 13.069;
    }
  }

  public static class IntakeConstants { //for claw
    // public static final double tickFactor =  Constants.CPR / 360;
    // public static final double intakeMargin = 3;

    public static final double DEFAULT_WRIST_ANGLE = 16.5;
    public static final double SUPPORT_WRIST_ANGLE = 239; 
    public static final double INTAKE_DOUBLE_WRIST_ANGLE = 77.25; 
    public static final double SHOOT_WRIST_ANGLE = INTAKE_DOUBLE_WRIST_ANGLE; 
    public static final double INTAKE_SINGLE_WRIST_ANGLE = 118; //tbd
    public static final double SCORE_WRIST_ANGLE = INTAKE_SINGLE_WRIST_ANGLE; 


    // public static final double WRIST_SPEED = 0.1;
    public static final int CLAW_MOTOR_CURRENT_LIMIT = 20; // amps
    public static final int WRIST_MOTOR_CURRENT_LIMIT = 30; // amps
    // public static final int MAX_CLAW_CURRENT = 25; //amps
    public static final int CLAW_MOTOR_SECONDARY_LIMIT = 35; // amps
    public static final int WRIST_MOTOR_SECONDARY_LIMIT = 35; // amps

    public final static double wristkP = 0.07;
    public final static double wristkI = 0.0;
    public final static double wristkD = 0.0;
  }

  public static final class DriveConstants {

    public static final double MAX_SPEED = 4.8; // max speed meters per second *** LOOK INTO MAX ALLOWED SPEED
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // slew rate constants
    public static final double DIR_SLEW_RATE = 1.2; // radians per second
    public static final double MAG_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    public static final double ROT_SLEW_RATE = 2.0; // percent per second (1 = 100%)

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

    // angular offsets of the modules relative to the chassis in radians
    public static final double FL_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FR_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double RL_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double RR_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    public static final boolean GYRO_REVERSED = false;
  }

  public static final class ModuleConstants {

    // 14T pinion gears
    public static final int DRIVE_MOTOR_PINION_TEETH = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER = 0.0762; // meters
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 35; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT = 35; // amps
    
    public static final IdleMode kDriveMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final class Drive {
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

      public static final double kP = 0.03; // initially 0.04
      public static final double kI = 0;
      public static final double kD = 0.01;
      // public static final double kS = 0.0; // placeholder --> run sysid
      // public static final double kA = 0.0; // placeholder 
      // public static final double kV = 0.0; //placeholder
      public static final double kFF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
    }

    public static final class Turning {
      // Invert the turning encoder, since the output shaft rotates in the opposite direction of
      // the steering motor in the MAXSwerve Module.
      public static final boolean ENCODER_INVERTED = true;
      // turning encoder position factor
      public static final double ENCODER_PFACTOR = (2 * Math.PI); // radians
      // velocity factor
      public static final double ENCODER_VFACTOR = (2 * Math.PI) / 60.0; // radians per second

      // turning encoder position pid min input
      public static final double ENCODER_PPID_MIN = 0; // radians
      // max input
      public static final double ENCODER_PPID_MAX = ENCODER_PFACTOR; // radians

      public static final double kP = 0.15; // initally 1
      public static final double kI = 0;
      public static final double kD = 0.05;
      public static final double kFF = 0;
      public static final double kMinOutput = -1;
      public static final double kMaxOutput = 1;
    }
  }

  public static final class AutoConstants {
    // drivetrain
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

    // intake wheels
    public static final double AUTON_OUTTAKE_TIME = 2;

    // wrist angles
    public static final double SUPPORT_WRIST_ANGLE_TIME = 2.3; // CHANGE AFTER TESTING was 1
    public static final double SCORE_WRIST_ANGLE_TIME = 0.95;

    // arm angles
    public static final double AUTON_INC_ARM_ANGLE_TIME = 2.5; // CHANGE AFTER TESTING
    public static final double AUTON_DEC_ARM_ANGLE_TIME = 3.4;

    // arm extensions
    public static final double AUTON_EXTEND_MID_ARM_TIME = 0.2375;
    public static final double AUTON_EXTEND_HIGH_ARM_TIME = 1.2;
    // public static final double AUTON_RETRACT_DEFAULT_ARM_TIME = 3.0;

    // charge
    public static final double AUTON_CHARGE_TIME = 3.0;
    public static final double AUTON_CHARGE_SPEED = 3.0;
    public static final double AUTON_TAXICHARGE_TIME = 2.0;
   
    // taxi time
    public static final double CHARGE_TIME = 3.475; // test run was 5.44 sec time needed to engage, starting from center
    public static final double LONG_TAXI_TIME = 4; // robot is close to charge station but not on it
    public static final double SHORT_TAXI_TIME = 3;
    //public static final double NODE_TO_GP_TIME = CHARGE_TIME + 2.0; // drive thru charge station

    // auton drive speeds
    public static final double TAXI_SPEED = 0.185;
    public static final double CHARGE_SPEED = 0.185;
    public static final double GP_SCORE_TIME = 2.0; // how long gp takes to fall out of intake and onto node
  }

  public static final class NeoMotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  }
}