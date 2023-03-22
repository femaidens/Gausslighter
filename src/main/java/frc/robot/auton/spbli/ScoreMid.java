// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.spbli;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.auton.spbli.arm.AutonExtendArm;
import frc.robot.auton.spbli.arm.AutonIncArmAngle;
import frc.robot.auton.spbli.arm.AutonRetractArm;
import frc.robot.auton.spbli.arm.AutonDecArmAngle;
import frc.robot.auton.spbli.wrist.AutonDecWristAngle;
import frc.robot.auton.spbli.wrist.AutonIncWristAngle;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.arm.RetractArm;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.intake2.CloseClaw2;
import frc.robot.commands.wrist.DecreaseWristAngle;
import frc.robot.commands.wrist.IncreaseWristAngle;
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
        new AutonIncWristAngle(intake, AutoConstants.AUTON_INC_WRIST_ANGLE_TIME, IntakeConstants.SUPPORT_WRIST_ANGLE),
        Commands.parallel(
            new AutonExtendArm(armLateral, PositionConfig.midLength),
            new AutonIncArmAngle(armAngle, PositionConfig.midNodeAngle)),
        // new StartEndCommand(
        // () -> intake.setWristAngle(IntakeConstants.INTAKE_WRIST_ANGLE), () ->
        // intake.openClaw()),
        new WaitCommand(0.5),
        new SequentialCommandGroup(
            // decreasing because we start at support wrist angle (largest angle)
            new AutonDecWristAngle(intake, AutoConstants.AUTON_DEC_WRIST_ANGLE_TIME, IntakeConstants.SCORE_WRIST_ANGLE),
            new OpenClaw(intake)),
        // new WaitCommand(AutoConstants.GP_SCORE_TIME), //wait for piece to fall onto
        // node

        // resetting robot config
        new AutonRetractArm(armLateral, AutoConstants.AUTON_RETRACT_DEFAULT_ARM_TIME),
        new AutonIncArmAngle(armAngle, PositionConfig.defaultAngle)
    // new CloseClaw2(intake)
    );
  }
}
