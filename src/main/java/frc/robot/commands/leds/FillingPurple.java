// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class FillingPurple extends CommandBase {
  private final LED led;
  private final Timer timer;
  /** Creates a new FillingPurple. */
  public FillingPurple(LED led, Timer timer) {
    this.led = led;
    this.timer = timer;
    addRequirements(led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.fillingPurple(timer);
  }

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
