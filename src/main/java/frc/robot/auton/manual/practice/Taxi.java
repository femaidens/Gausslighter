// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.manual.practice;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ArmConstants.PositionConfig;
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
public class Taxi extends SequentialCommandGroup {
  public Taxi(Drivetrain drivetrain) {
    // starting @ left/right, facing gp
    addCommands(

          new AutonDrive(drivetrain, 0, AutoConstants.SCORE_AND_ENGAGE_SPEED, 0, 
            true, true, AutoConstants.NODE_TO_CHARGE_TIME));
  }
}
