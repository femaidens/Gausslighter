// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonRoutines.mid;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.autonScore.ScoreMid;
import frc.robot.auton.autonTaxi.ShortTaxi;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class ScoreMidShortTaxi extends SequentialCommandGroup {
  //starting @ left/right, facing nodes 
  public ScoreMidShortTaxi(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // usable for all start positions in community (assuming you're aligned with a gamepiece outside of community)
    addCommands(
      // score high
      new ScoreMid(intake, armAngle, armLateral),
      new ShortTaxi(drivetrain)
    );
  }
}