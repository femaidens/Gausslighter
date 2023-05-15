// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonDrivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutonDrive extends CommandBase {

  private final Drivetrain drivetrain;
  private final double xSpeed;
  private final double ySpeed;
  private final double rot;
  private final boolean fieldRelative;
  private final boolean rateLimit;
  private final double driveTime; // how much time robot should drive for
  private final Timer timer = new Timer();

  public AutonDrive(
      Drivetrain drivetrain,
      double xSpeed,
      double ySpeed,
      double rot,
      boolean fieldRelative,
      boolean rateLimit,
      double driveTime) {

    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    this.fieldRelative = fieldRelative;
    this.rateLimit = rateLimit;
    this.driveTime = driveTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= driveTime); // check if desired drive time has been reached, if so then stop driving
  }
}
