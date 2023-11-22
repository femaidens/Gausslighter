// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class GreenLEDS extends CommandBase {
  private final LED led;
  private final Timer timer;
  private final double time; //seconds
  /** Creates a new GreenLEDS. */
  public GreenLEDS(LED led, double t) {
    this.led = led;
    this.time = t;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    led.greenLED();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
