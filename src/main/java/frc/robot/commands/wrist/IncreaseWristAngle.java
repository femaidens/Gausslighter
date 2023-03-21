// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IncreaseWristAngle extends CommandBase {
  /** Creates a new SetWristAngle. */
  private final Intake intake;
  private double wristAngle;
  
  // use increase angle from default -> intake; intake -> support; default -> support
  public IncreaseWristAngle(Intake intake, double wristAngle) {
    this.intake = intake;
    this.wristAngle = wristAngle;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.increaseWristAngle(wristAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWristMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.atWristAngle(wristAngle);
  }
}
