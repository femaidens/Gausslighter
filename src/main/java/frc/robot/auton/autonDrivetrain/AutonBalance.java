// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonDrivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonBalance extends CommandBase {
  /** Creates a new AutonDrive. */
  private final Drivetrain drivetrain;
  private final double xSpeed;
  private final double ySpeed;
  private final double rot;
  private final boolean fieldRelative;
  private final boolean rateLimit;
  private double pitch;

  public AutonBalance(
      Drivetrain drivetrain,
      double xSpeed,
      double ySpeed,
      double rot,
      boolean fieldRelative,
      boolean rateLimit) {

    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    this.fieldRelative = fieldRelative;
    this.rateLimit = rateLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitch = drivetrain.gyroX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = drivetrain.gyroX(); // gives us the pitch
    drivetrain.drive(xSpeed * Math.signum(pitch), 0, 0, fieldRelative, rateLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pitch) < 13; // check if desired drive time has been reached, if so then stop driving
  }
}
