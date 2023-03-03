// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetClawAngle extends CommandBase {
  /** Creates a new setClawAngle. */
  public final Intake intake; 
  public final double angle;
  /** Creates a new resetClawAngle. */
  public SetClawAngle(Intake intake, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.angle = angle;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setWristAnglePID(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWristMotor(); // or can we just do intake.wrist.set(0); ??
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}