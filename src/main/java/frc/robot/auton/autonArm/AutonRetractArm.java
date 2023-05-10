// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmLateral;

public class AutonRetractArm extends CommandBase {
  /** Creates a new AutonRetractArm. */
  private final ArmLateral armLateral;
  private double retractArmTime;
  private final Timer timer = new Timer();

  public AutonRetractArm(ArmLateral armLateral, double retractArmTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armLateral = armLateral;
    this.retractArmTime = retractArmTime;
    addRequirements(armLateral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // while (timer.get() != driveTime) {
      // if (timer.get() < retractArmTime) {
        armLateral.retractArm(false);
      //}
      // else {
      //   armLateral.stopExtensionMotors();
      // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armLateral.stopExtensionMotors();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= retractArmTime);
  }
}
