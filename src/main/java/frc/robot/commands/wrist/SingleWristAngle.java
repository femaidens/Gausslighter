// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class SingleWristAngle extends CommandBase {
  /** Creates a new SetWristAngle. */
  private final Intake intake;
  private double wristAngle;
  private Timer timer;
  
  // use increase angle from default -> intake; intake -> support; default -> support
  public SingleWristAngle(Intake intake) {
    this.intake = intake;
    timer = new Timer();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runWristMotor(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWristMotor();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > IntakeConstants.DEFAULT_TO_SINGLE_TIME;
  }
}
