// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class AltGreenPurpleLEDS extends CommandBase {
  private final LED leds;
  private final Timer timer;
  // private final int x;
  //private boolean purple;
  /** Creates a new AltGreenPurpleLEDS. */
  public AltGreenPurpleLEDS(LED leds, Timer timer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    this.timer = timer;
    addRequirements(leds);
    //x = 0;
    //purple = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.lightShow(timer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // timer.reset();
    leds.showProgramCleanUp();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
