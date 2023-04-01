// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.spbli.autonWrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutonOuttake extends CommandBase {
  /** Creates a new AutonOuttake. */
  private final Intake intake;
  private Timer timer;
  private double outtakeTime;
  public AutonOuttake(Intake intake, double outtakeTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.outtakeTime = outtakeTime;
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
    intake.reverseIntakeMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= outtakeTime;
  }
}
