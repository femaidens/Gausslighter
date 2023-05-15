// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonRoutines.mid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.auton.autonArm.*;
import frc.robot.auton.autonRoutines.TaxiCharge;
import frc.robot.auton.autonScore.ScoreHigh;
import frc.robot.auton.autonWrist.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// parallel cmd 1
//1: 3 cmds, change wr ang from def to support ang
// 2: ext arm from def to mid node
// 3 def arm ---> dec until mid node
// sequential cmd
// lower wrist - support to score wrist
// 2: open claw

public class ScoreMidTaxiCharge extends SequentialCommandGroup {
  /** Creates a new ScoreMid. */
  public ScoreMidTaxiCharge(Intake intake, ArmAngle armAngle, ArmLateral armLateral, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ScoreHigh(intake, armAngle, armLateral),
        new TaxiCharge(drivetrain));

      // sbpli version
        //new AutonIncWristAngle(intake, AutoConstants.SUPPORT_WRIST_ANGLE_TIME));
        // new ParallelCommandGroup(
        // // new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME),
        // new AutonDecArmAngle(armAngle)
        // ),
        // new PrintCommand("parallel ended"));
        // new AutonDecWristAngle(intake, AutoConstants.SCORE_WRIST_ANGLE_TIME),
        // new RunCommand(() -> intake.openClaw(), intake));
        
    // new CloseClaw2(intake)
  }
}
