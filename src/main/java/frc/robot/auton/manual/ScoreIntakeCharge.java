// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.manual;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Constants.*;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.Intake2.CloseClaw2;
import frc.robot.commands.Intake2.RunIntake;
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
public class ScoreIntakeCharge extends SequentialCommandGroup {
  /** Creates a new ScoreIntakeCharge. */
  public ScoreIntakeCharge(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // score high
      new SetArmAngle(armAngle, PositionConfig.highNodeAngle),
      new ExtendArm(armLateral, PositionConfig.highLength),
      new OpenClaw(intake),
      new WaitCommand(3.0), //wait for piece to fall onto node (change wait time after testing)

      // move toward gamepiece through charge station
      new RetractArm(armLateral, PositionConfig.defaultAngle),
      new SetArmAngle(armAngle, PositionConfig.defaultAngle),
      new AutonDrive(drivetrain, -AutoConstants.SCORE_AND_CHARGE_SPEED, 0, 0, true, true, 
      AutoConstants.SCORE_AND_CHARGE_TIME + 2.0), //move backward enough so that arm doesn't hit game piece when rotating

      // turn around to intake
      new AutonDrive(drivetrain, 0, 0, 180, true, true, 1.0),
      new AutonDrive(drivetrain, AutoConstants.CHARGE_SPEED, 0, 0, true, true, 1.0), 
      //move forward slightly to get to gamepiece, claw is already open from scoring
      new RunIntake(intake),
      new CloseClaw2(intake),

      //move backward onto charge station
      new AutonDrive(drivetrain, -0.1, 0, 0, true, true, 1.5)
    );
  }
}