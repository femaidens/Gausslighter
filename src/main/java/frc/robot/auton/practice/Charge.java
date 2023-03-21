// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.practice;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.*;
import frc.robot.commands.AutonDrive;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Charge extends SequentialCommandGroup {
  public Charge(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // SET UP ROBOT TO FACE CHARGE UP
    // starting on the center position; right in front and facing charge up
    addCommands(
      new AutonDrive(drivetrain, 0, AutoConstants.SCORE_AND_ENGAGE_SPEED, 0, true, true, 
      AutoConstants.NODE_TO_CHARGE_TIME)
    );
  }
}