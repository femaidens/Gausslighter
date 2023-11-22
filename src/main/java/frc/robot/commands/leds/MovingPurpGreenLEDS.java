// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LED;

public class MovingPurpGreenLEDS extends CommandBase {
  private final LED led;
  private final Timer timer;
  private int space = 0;
  /** Creates a new MovingPurpGreenLEDS. */
  public MovingPurpGreenLEDS(LED led) {
    this.led = led;
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
    if(timer.hasElapsed(0.2)){
      timer.reset();
      space ++;
    } 
    else {
      led.movingPurpGreenLED(space);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return space == LEDConstants.LED_PIN_LENGTH-10;
  }
}
