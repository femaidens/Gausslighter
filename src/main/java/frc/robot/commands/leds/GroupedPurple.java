// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class GroupedPurple extends CommandBase {
  /** Creates a new GroupedPurple. */
  private final LED leds;
  private final Timer timer;
  public GroupedPurple(LED leds, Timer timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    this.timer = timer;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.groupedPurple(timer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.showProgramCleanUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
