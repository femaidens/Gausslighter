// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.practice;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Constants.*;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.arm.ExtendArm;
import frc.robot.commands.arm.RetractArm;
import frc.robot.commands.arm.SetArmAngle;
import frc.robot.commands.intake2.CloseClaw2;
import frc.robot.commands.wrist.DecreaseWristAngle;
import frc.robot.commands.wrist.IncreaseWristAngle;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score extends SequentialCommandGroup {
  //starting @ left/right, facing nodes 

  public Score(Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // usable for all start positions in community (assuming you're aligned with a gamepiece outside of community)
    addCommands(
      // score high
      new IncreaseWristAngle(intake, IntakeConstants.SUPPORT_WRIST_ANGLE),
      Commands.parallel(
        new SetArmAngle(armAngle, PositionConfig.highNodeAngle),
        new ExtendArm(armLateral, PositionConfig.highLength)
      ),
      // new StartEndCommand(
      //   () -> intake.setWristAngle(IntakeConstants.INTAKE_WRIST_ANGLE), () -> intake.openClaw()),
      new WaitCommand(0.5),
        new SequentialCommandGroup(
          // decreasing because we start at support wrist angle (largest angle)
          new DecreaseWristAngle(intake, IntakeConstants.SCORE_WRIST_ANGLE),
          new OpenClaw(intake)
      ),
      // new WaitCommand(AutoConstants.GP_SCORE_TIME), //wait for piece to fall onto node

      // resetting robot config
      new RetractArm(armLateral, PositionConfig.defaultLength),
      new SetArmAngle(armAngle, PositionConfig.midNodeAngle),
      new CloseClaw2(intake)
    );
  }
}