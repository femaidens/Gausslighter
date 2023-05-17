// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class ConeLEDS extends CommandBase{
  
  private final LED led;
  private final Timer timer;
  
  public ConeLEDS(LED led) {
    this.led = led;
    addRequirements(led);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  public boolean atTime(double secs){
    if (timer.get() > secs) return true;
    return false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.ConeLED();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    led.showProgramCleanUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atTime(5);
  }
}
