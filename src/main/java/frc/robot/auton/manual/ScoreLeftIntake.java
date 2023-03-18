// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.manual;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.Intake2.CloseClaw2;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.RetractArm;
import frc.robot.commands.SetArmAngle;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLeftIntake extends SequentialCommandGroup {
  /** Creates a new ScoreLeftIntake. */
  public ScoreLeftIntake(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    
    // left starting position (relative to the driver station), 
    // ALLIANCE COLOR --> true = blue and false = red

      addCommands(
      new SetArmAngle(armAngle, PositionConfig.highConeAngle),
      new ExtendArm(armLateral, PositionConfig.highLength),
      new OpenClaw(intake),
      new WaitCommand(3.0), //wait for piece to fall onto node & change after testing
      new RetractArm(armLateral, PositionConfig.defaultAngle),
      new SetArmAngle(armAngle, PositionConfig.defaultAngle),
      new AutonDrive(drivetrain, 0, 0, 90, true, true, 2.0), 
      //robot turns to its right to face wall
      new AutonDrive(drivetrain, 0, 0, 90, true, true, 2.0),
      //robot turns to its right again to face game pieces
      new AutonDrive(drivetrain, AutoConstants.SCORE_AND_CHARGE_SPEED, 0, 0, true, true, 4.0),
      //drive forward to game piece, placeholder speed
      new CloseClaw2(intake)
      );
    }
 
  }
  

