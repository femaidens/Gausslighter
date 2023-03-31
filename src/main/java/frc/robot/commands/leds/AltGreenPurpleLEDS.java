// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED2;

public class AltGreenPurpleLEDS extends CommandBase {
  private final LED2 leds;
  private final Timer timer;
  private final int x;
  private final boolean purple;
  /** Creates a new AltGreenPurpleLEDS. */
  public AltGreenPurpleLEDS() {
    // Use addRequirements() here to declare subsystem dependencies.
    leds = new LED2();
    timer = new Timer();
    addRequirements(leds);
    x = 0;
    purple = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(purple){
      leds.altPurpGreen();
    } else {
      //wip
    }
      

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    leds.offLEDS();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(5);
  }
}
