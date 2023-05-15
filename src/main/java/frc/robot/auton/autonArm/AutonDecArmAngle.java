// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmAngle;

public class AutonDecArmAngle extends CommandBase {
  private final ArmAngle armAngle;
  private Timer timer = new Timer();

  public AutonDecArmAngle(ArmAngle armAngle) {
    this.armAngle = armAngle;
    addRequirements(armAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //currAngle = armAngle.getArmAngle();
    //if (currAngle > goalAngle) {
      armAngle.decreaseArmAngle();
      System.out.println("running arm angle motor");
      System.out.println("current angle" + armAngle.getArmAngle());
    //}
    // else {
    //   armAngle.stopAngleMotor();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ending arm angle motor");
    armAngle.stopAngleMotor();
    timer.reset();
  }
// sus

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > AutoConstants.AUTON_DEC_ARM_ANGLE_TIME;
  }
}
