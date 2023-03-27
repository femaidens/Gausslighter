// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DecreaseWristAngle extends CommandBase {

  private final Intake intake;
  private double angle;
  private Timer wristTimer;

  // use decrease angle from support -> score; support -> intake; support -> default; intake -> default
  public DecreaseWristAngle(Intake intake, double angle) {
    this.intake = intake;
    this.angle = angle;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (wristTimer.get() < 0.9){
      intake.decreaseWristAngle(angle);
    }
    intake.stopWristMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristTimer.reset();
    intake.stopWristMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristTimer.get() >= 0.9;
  }
}
