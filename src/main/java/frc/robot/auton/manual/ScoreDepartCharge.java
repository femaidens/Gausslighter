// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.manual;

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
public class ScoreDepartCharge extends SequentialCommandGroup {
  // assuming center start position and drives through charge station, CAN BE USED FOR EITHER ALLIANCE COLOR

  public ScoreDepartCharge(Drivetrain drivetrain, Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmAngle(armAngle, PositionConfig.highConeAngle),
      new ExtendArm(armLateral, PositionConfig.highLength),
      new OpenClaw(intake),
      new WaitCommand(3.0), //wait for piece to fall onto node
      new CloseClaw2(intake),
      new RetractArm(armLateral, PositionConfig.defaultAngle),
      new AutonDrive(drivetrain, -0.25, 0, 0, true, true, AutoConstants.SCORE_AND_CHARGE_TIME + 3.0),
      // drive backward for a longer time to move out of community
      new AutonDrive(drivetrain, AutoConstants.CHARGE_SPEED, 0, 0, true, true, 0.5)
      // drive forward at a slower speed for less time to engage on station
    );
    
  }
}
