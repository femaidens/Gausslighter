// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonWrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutonSetWristAngle extends CommandBase {
  private final Intake intake;
  private double autonSetpoint;
  
  public AutonSetWristAngle(Intake intake, double setpoint) {
    this.intake = intake;
    autonSetpoint = setpoint;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("setting wrist angle");
    intake.setAutonWristAngle(autonSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWristMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.atWristAngle(autonSetpoint);
  }
}
