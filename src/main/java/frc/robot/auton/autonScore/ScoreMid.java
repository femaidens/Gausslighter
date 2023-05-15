// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonScore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.auton.autonArm.*;
import frc.robot.auton.autonWrist.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Intake;

// parallel cmd 1
//1: 3 cmds, change wr ang from def to support ang
// 2: ext arm from def to mid node
// 3 def arm ---> dec until mid node
// sequential cmd
// lower wrist - support to score wrist
// 2: open claw

public class ScoreMid extends SequentialCommandGroup {
  public ScoreMid(Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    SequentialCommandGroup score = new SequentialCommandGroup(
      new ParallelCommandGroup(
            new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
            new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_MID_ARM_TIME), //test for this time
            new AutonDecArmAngle(armAngle)
          ),
          new InstantCommand(() -> intake.openClaw(), intake)
    );

    addCommands(
      score.withTimeout(5),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new InstantCommand(
          () -> intake.closeClaw(), intake),
        new AutonRetractArm(armLateral, AutoConstants.AUTON_EXTEND_MID_ARM_TIME - 0.01)
      )
      // score
    );
    // addCommands(

    //   // pid
    //   new ParallelCommandGroup(
    //     new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
    //     new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_MID_ARM_TIME),
    //     new AutonSetArmAngle(armAngle, PositionConfig.midNodeAngle)
    //     ),
    //   // new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
    //   new RunCommand(() -> intake.openClaw(), intake));
   
    // new CloseClaw2(intake)

  }
}
