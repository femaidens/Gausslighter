// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonRoutines.high;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.autonRoutines.Charge;
import frc.robot.auton.autonScore.ScoreHigh;
import frc.robot.auton.autonScore.ScoreMid;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreHighCharge extends SequentialCommandGroup {
  /** Creates a new ScoreMidCharge. */

  public ScoreHighCharge(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ScoreHigh(intake, armAngle, armLateral),
      new Charge(drivetrain, AutoConstants.SLOWCHARGE_SPEED, AutoConstants.BACKCHARGE_TIME)
    );
  }
}
