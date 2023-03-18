// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.CloseClaw2;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.SetArmAngle;
import frc.robot.commands.SetArmExtension;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAndIntake extends SequentialCommandGroup {
  public ScoreAndIntake(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    addCommands(
      new SetArmAngle(armAngle, PositionConfig.highConeAngle, true),
      new SetArmExtension(armLateral, PositionConfig.highLength),
      new OpenClaw(intake),
      new WaitCommand(3.0), //wait for piece to fall onto node & change after testing
      new SetArmExtension(armLateral, PositionConfig.defaultAngle),
      new SetArmAngle(armAngle, PositionConfig.defaultAngle, false),
      new AutonDrive(drivetrain, -AutoConstants.SCORE_AND_CHARGE_SPEED, 0, 0, true, true, 
      AutoConstants.SCORE_AND_CHARGE_TIME + 2.0), //move backward enough so that arm doesn't hit game piece when rotating
      new AutonDrive(drivetrain, 0, 0, 180, true, true, 1.0),
      new AutonDrive(drivetrain, 0.05, 0, 0, true, true, 1.0), 
      //move forward slightly to get to gamepiece, claw is already open from scoring
      new CloseClaw2(intake)
    );
  }
}
