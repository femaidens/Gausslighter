// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonScore;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.auton.autonArm.*;
import frc.robot.auton.autonWrist.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// parallel cmd 1
//1: 3 cmds, change wr ang from def to support ang
// 2: ext arm from def to mid node
// 3 def arm ---> dec until mid node
// sequential cmd
// lower wrist - support to score wrist
// 2: open claw

public class ScoreMid extends SequentialCommandGroup {
  /** Creates a new ScoreMid. */
  public ScoreMid(Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      // pid
      new ParallelCommandGroup(
        new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
        new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_MID_ARM_TIME),
        new AutonSetArmAngle(armAngle, PositionConfig.midNodeAngle)
        ),
      // new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
      new RunCommand(() -> intake.openClaw(), intake));

      // sbpli
      // new AutonIncWristAngle(intake, AutoConstants.SUPPORT_WRIST_ANGLE_TIME),
      // new ParallelCommandGroup(
      // new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_MID_ARM_TIME),
      // new AutonDecArmAngle(armAngle)
      // ),
      // new AutonDecWristAngle(intake, AutoConstants.SCORE_WRIST_ANGLE_TIME),
      // new RunCommand(() -> intake.openClaw(), intake));

      // // resetting robot config
      // new AutonRetractArm(armLateral, AutoConstants.AUTON_RETRACT_DEFAULT_ARM_TIME),
      // new AutonIncArmAngle(armAngle, PositionConfig.defaultAngle)
    // new CloseClaw2(intake)

  }
}
