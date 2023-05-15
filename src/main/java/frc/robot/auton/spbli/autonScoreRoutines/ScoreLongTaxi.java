// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.spbli.autonScoreRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.spbli.autonScoreCmds.ScoreMid;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLongTaxi extends SequentialCommandGroup {
  //starting @ left/right, facing nodes 
  public ScoreLongTaxi(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // usable for all start positions in community (assuming you're aligned with a gamepiece outside of community)
    addCommands(
      // score high
      new ScoreMid(intake, armAngle, armLateral),
      new LongTaxi2(drivetrain)
    );
  }
}