// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class AutonDrive extends CommandBase {
  /** Creates a new AutonDrive. */
  private final Drivetrain drivetrain;
  private Object SCORE_AND_CHARGE;
  private List<PathPoint> points;
  //private final SwerveDriveKinematics kinematics;
  private final PIDController xPIDController;
  private final PIDController yPIDController;
  private final PIDController rotPIDController;

  public AutonDrive(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    //this.trajectory = trajectory;

    xPIDController = new PIDController(
          ArmConstants.AnglePID.kP,
          ArmConstants.AnglePID.kI,
          ArmConstants.AnglePID.kD);
      yPIDController = new PIDController(
          ArmConstants.AnglePID.kP,
          ArmConstants.AnglePID.kI,
          ArmConstants.AnglePID.kD);
      rotPIDController = new PIDController(
          ArmConstants.AnglePID.kP,
          ArmConstants.AnglePID.kI,
          ArmConstants.AnglePID.kD);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathConstraints trajectoryConstraints = new PathConstraints(
              DriveConstants.MAX_VELOCITY_CHASSIS,
              DriveConstants.MAX_ACCEL_CHASSIS);

  PathPoint start = new PathPoint(DriveConstants.startPos, new Rotation2d(0.0));
  PathPoint end = new PathPoint(new Translation2d(3.91, 3.30), new Rotation2d(0.0));

  points.add(start);
  points.add(end); // ading pathpoints into pathplanner path

    this.SCORE_AND_CHARGE = PathPlanner.generatePath(
              trajectoryConstraints, false, points);
      System.out.println("Trajectory successfully generated!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(-0.25, 0, 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
