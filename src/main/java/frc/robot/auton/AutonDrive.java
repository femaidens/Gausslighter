// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonDrive extends CommandBase {
  /** Creates a new AutonDrive. */
  private final Drivetrain drivetrain;
  private final double xSpeed;
  private final double ySpeed;
  private final double rot;
  private final boolean fieldRelative;
  private final boolean rateLimit;

  public AutonDrive(
    Drivetrain drivetrain, 
    double xSpeed, 
    double ySpeed, 
    double rot, 
    boolean fieldRelative, 
    boolean rateLimit)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rot = rot;
    this.fieldRelative = fieldRelative;
    this.rateLimit = rateLimit;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit); 
    // may need to change rot value to get onto charge station
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
