// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class BouncingPurple extends CommandBase {
  private final LED led;
  /** Creates a new BouncingPurple. */
  public BouncingPurple(LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(
    led.bouncingPurp();
  ) {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.showProgramCleanUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
