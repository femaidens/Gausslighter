// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.spbli.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAngle;

public class AutonDecArmAngle extends CommandBase {
  /** Creates a new AutonSetArmAngle. */
  private final ArmAngle armAngle;
  private double goalAngle;
  private double currAngle;

  public AutonDecArmAngle(ArmAngle armAngle, double goalAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armAngle = armAngle;
    this.goalAngle = goalAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currAngle = armAngle.getArmAngle();
    if (currAngle > goalAngle) {
      armAngle.decreaseArmangle(goalAngle);
    }
    else {
      armAngle.stopAngleMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armAngle.stopAngleMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currAngle >= goalAngle;
  }
}
