// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.manual;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Constants.*;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.RetractArm;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.Intake2.CloseClaw2;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreDriveToGP extends SequentialCommandGroup {
  public ScoreDriveToGP(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // can be used for every starting position in the community (assuming you're aligned with the gamepieces)
    addCommands(
      // score high
      new SetArmAngle(armAngle, PositionConfig.highNodeAngle),
      new ExtendArm(armLateral, PositionConfig.highLength),
      new OpenClaw(intake),
      new WaitCommand(AutoConstants.GP_SCORE_TIME), //wait for piece to fall onto node

      // drive backwards to leave community and towards gamepiece
      new RetractArm(armLateral, PositionConfig.defaultAngle),
      new SetArmAngle(armAngle, PositionConfig.defaultAngle),
        Commands.parallel(
          new CloseClaw2(intake),
          new AutonDrive(drivetrain, -0.25, 0, 0, true, true, 
            AutoConstants.SCORE_AND_CHARGE_TIME + 3.0))
    );
  }
}