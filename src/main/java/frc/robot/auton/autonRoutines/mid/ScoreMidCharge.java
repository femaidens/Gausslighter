// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonRoutines.mid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.autonRoutines.Charge;
import frc.robot.auton.autonScore.ScoreMid;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class ScoreMidCharge extends SequentialCommandGroup {

  public ScoreMidCharge(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    addCommands(
      new ScoreMid(intake, armAngle, armLateral),
      new Charge(drivetrain, AutoConstants.BACKCHARGE_SPEED, AutoConstants.INITIAL_RAMP_TIME)
    );
  }
}
