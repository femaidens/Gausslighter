// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmLateral;

public class ExtendArm extends CommandBase {
  /** Creates a new ExtendArm. */
  public final ArmLateral armLateral;
  private double position;

  public ExtendArm(ArmLateral armLateral, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armLateral = armLateral;
    this.position = position;
    addRequirements(armLateral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armLateral.extendArm();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armLateral.stopExtensionMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armLateral.atLength(position);
  }
}
