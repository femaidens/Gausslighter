// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final ArmLateral armLat;
  private final ArmAngle armAng;
  private final ArrayList TrajectoryPointsList;
  private Object trajectory1;
  
  public FollowTrajectory(Drivetrain drivetrain, Intake intake, ArmLateral armLat, ArmAngle armAng, ArrayList TrajectoryPointsList) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.armLat = armLat;
    this.armAng = armAng;
    this.TrajectoryPointsList = new ArrayList<PathPoint>();
  
    this.addRequirements(drivetrain, intake, armLat, armAng);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathConstraints trajectoryConstraints = new PathConstraints(
				DriveConstants.MAX_VELOCITY_CHASSIS,
				DriveConstants.MAX_ACCEL_CHASSIS);

    TrajectoryPointsList.add(new PathPoint(this.drivetrain.getPose().getTranslation(),
    // Calculate the angle between the robot's current point and the next
    new Rotation2d(this.getAngleFromPoints(
        this.drivetrain.getPose().getX(),
        this.drivetrain.getPose().getY(),
        DriveConstants.kTrajectoryEndPoseXFieldRelativeM,
        DriveConstants.kTrajectoryEndPoseYFieldRelativeM)),
    this.drivetrain.getPose().getRotation()));

    TrajectoryPointsList.add(new PathPoint(
      new Translation2d(
          DriveConstants.kTrajectoryEndPoseXFieldRelativeM,
          DriveConstants.kTrajectoryEndPoseYFieldRelativeM),
      // Calculate the angle between the robot's previous point and this one
      new Rotation2d(180 - this.getAngleFromPoints(
          this.drivetrain.getPose().getX(),
          this.drivetrain.getPose().getY(),
          DriveConstants.kTrajectoryEndPoseXFieldRelativeM,
          DriveConstants.kTrajectoryEndPoseYFieldRelativeM)),
      Rotation2d.fromDegrees(DriveConstants.kTrajectoryEndAngleFieldRelativeDeg)));

      this.trajectory1 = PathPlanner.generatePath(
				trajectoryConstraints,
				this.TrajectoryPointsList.get(0),
		System.out.println("Trajectory successfully generated!");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private double getAngleFromPoints(double x1, double y1, double x2, double y2) {
		return Math.PI / 2 - Math.atan2(y2 - y1, x2 - x1);
	}
}
