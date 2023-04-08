// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonScore;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

public class ScoreHigh extends SequentialCommandGroup {
  /** Creates a new ScoreHigh. */
  public ScoreHigh(Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SequentialCommandGroup score = new SequentialCommandGroup(
      new ParallelCommandGroup(
            new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
            new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME),
            new AutonDecArmAngle(armAngle)
          ),
          new InstantCommand(() -> intake.openClaw(), intake)
    );

    addCommands(
      score.withTimeout(2.5),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new InstantCommand(
          () -> intake.closeClaw(), intake),
        new AutonRetractArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME - 0.01)
      )
      //score
    );



      // // race command
      // new ParallelCommandGroup(
      //   // new AutonSetArmLength(armLateral, PositionConfig.highLength),
      //   new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
      //   new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME),
      //   new AutonDecArmAngle(armAngle)
      // //   new AutonSetArmAngle(armAngle, PositionConfig.highNodeAngle)
      // ),
      // //   new WaitCommand(1),
      // //   new AutonSetArmAngle(armAngle, PositionConfig.defaultAngle)
      // new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
      // new InstantCommand(() -> intake.openClaw(), intake),
      // new WaitCommand(0.5),
      // new ParallelCommandGroup(
      //   new InstantCommand(
      //     () -> intake.closeClaw(), intake),
      //   new AutonRetractArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME - 0.01))
      // );

      // // pid
      // addCommands(
      // new ParallelCommandGroup(
      //   // new AutonSetArmLength(armLateral, PositionConfig.highLength),
      //   new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
      //   new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME),
      //   new AutonDecArmAngle(armAngle)
      // //   new AutonSetArmAngle(armAngle, PositionConfig.highNodeAngle)
      // ),
      // //   new WaitCommand(1),
      // //   new AutonSetArmAngle(armAngle, PositionConfig.defaultAngle)
      // // new AutonSetWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
      // new InstantCommand(() -> intake.openClaw(), intake),
      // new WaitCommand(0.5),
      // new ParallelCommandGroup(
      //   new InstantCommand(
      //     () -> intake.closeClaw(), intake),
      //   new AutonRetractArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME - 0.01))
      // );

      // sbpli version
        //new AutonIncWristAngle(intake, AutoConstants.SUPPORT_WRIST_ANGLE_TIME));
        // new ParallelCommandGroup(
        // // new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME),
        // new AutonDecArmAngle(armAngle)
        // ),
        // new PrintCommand("parallel ended"));
        // new AutonDecWristAngle(intake, AutoConstants.SCORE_WRIST_ANGLE_TIME),
        // new RunCommand(() -> intake.openClaw(), intake));

        // old code
        // new StartEndCommand(
        // () -> intake.setWristAngle(IntakeConstants.INTAKE_WRIST_ANGLE), () ->
        // intake.openClaw()),
        // new WaitCommand(0.5),
        // new SequentialCommandGroup(
        //     // decreasing because we start at support wrist angle (largest angle)
        //     new AutonDecWristAngle(intake, AutoConstants.AUTON_DEC_WRIST_ANGLE_TIME, IntakeConstants.SCORE_WRIST_ANGLE),
        //     new OpenClaw(intake)),
        // // new WaitCommand(AutoConstants.GP_SCORE_TIME), //wait for piece to fall onto
        // // node

        // // resetting robot config
        // new AutonRetractArm(armLateral, AutoConstants.AUTON_RETRACT_DEFAULT_ARM_TIME),
        // new AutonIncArmAngle(armAngle, PositionConfig.defaultAngle)
    // new CloseClaw2(intake)
  }
}
