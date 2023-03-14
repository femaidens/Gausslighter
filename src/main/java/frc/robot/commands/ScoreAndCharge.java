// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ScoreAndCharge extends CommandBase {
  /** Creates a new ScoreAndCharge. */

  private final Drivetrain drivetrain;
  private final ArmLateral armLateral;
  private final ArmAngle armAngle;

  private Object SCORE_AND_CHARGE;
  private List<PathPoint> points;

  public ScoreAndCharge(Drivetrain drivetrain, ArmLateral armLateral, ArmAngle armAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.armLateral = armLateral;
    this.armAngle = armAngle;
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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
