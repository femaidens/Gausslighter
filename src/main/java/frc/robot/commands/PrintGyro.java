// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gyro;

public class PrintGyro extends CommandBase {
  private Gyro m_gyro;
  /** Creates a new PrintGyro. */
  public PrintGyro(Gyro m_gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_gyro = m_gyro;
    addRequirements(m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_gyro.printValues();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
