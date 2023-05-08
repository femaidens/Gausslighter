// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class PurpGreenLEDS extends CommandBase {
  /** Creates a new PurpGreenLEDS. */
  private final LED led;
  private final Timer timer;
  public PurpGreenLEDS(LED led) {
    this.led = led;
    addRequirements(led);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    // new WaitCommand(5);
    // led.showProgramCleanUp();
    // led.rainbow();
  }

  // public boolean atTime(double sec){
  //   if (timer.get() > sec) return true;
  //   return false;
  // }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.purpGreen();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //timer.reset();
    led.showProgramCleanUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
