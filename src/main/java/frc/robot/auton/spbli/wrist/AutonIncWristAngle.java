// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.spbli.wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class AutonIncWristAngle extends CommandBase {
  /** Creates a new AutonIncWristAngle. */
  private final Intake intake;
  private final Timer timer = new Timer();
  private final double wristTime;
  private final double wristAngle;


  public AutonIncWristAngle(Intake intake, double wristTime, double wristAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.wristTime = wristTime;
    this.wristAngle = wristAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // while (timer.get() != driveTime) {
      // if (timer.get() < wristTime) {
        intake.increaseWristAngle(wristAngle);
      // }
      // else {
      //   intake.stopWristMotor();
      // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopWristMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= wristTime); // check if desired drive time has been reached, if so then stop driving
  }
}
