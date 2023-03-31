package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class CubeLEDS extends CommandBase {
  /** Creates a new coneLEDS. */
  private final LED led;
  private final Timer timer;

  public CubeLEDS(LED led) {
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
    led.CubeLED();
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
