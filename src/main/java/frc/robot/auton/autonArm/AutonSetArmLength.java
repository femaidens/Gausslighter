// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;

public class AutonSetArmLength extends CommandBase {
  private final ArmLateral armLateral;
  private double autonSetpoint;
  
  /** Creates a new AutonSetArmAngle. */
  public AutonSetArmLength(ArmLateral armLateral, double setpoint) {
    this.armLateral = armLateral;
    autonSetpoint = setpoint;
    addRequirements(armLateral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armLateral.setAutonArmLength(autonSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armLateral.stopExtensionMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
