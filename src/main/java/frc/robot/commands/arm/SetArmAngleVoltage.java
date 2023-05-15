// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAngle;

public class SetArmAngleVoltage extends CommandBase {
  private final ArmAngle armAngle;
  
  public SetArmAngleVoltage(ArmAngle armAngle) {
    this.armAngle = armAngle;
    addRequirements(armAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAngle.setAngleVoltage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armAngle.stopAngleMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armAngle.getIsManual();
  }
}
