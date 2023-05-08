// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonScore;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.auton.autonArm.*;
import frc.robot.auton.autonWrist.AutonOuttake;
import frc.robot.auton.autonWrist.AutonSetWristAngle;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootMid extends SequentialCommandGroup {
  /** Creates a new ShootMid. */
  public ShootMid(Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    // pid ver
    // new AutonSetWristAngle(intake, IntakeConstants.SUPPORT_WRIST_ANGLE),
    new ParallelCommandGroup(
      //new AutonSetArmLength(armLateral, PositionConfig.midLength),
      new AutonSetWristAngle(intake, IntakeConstants.SHOOT_WRIST_ANGLE),
      new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_MID_ARM_TIME),
      new AutonSetArmAngle(armAngle, PositionConfig.midNodeAngle)
      ),
      new AutonOuttake(intake, AutoConstants.AUTON_OUTTAKE_TIME)
    // first ver
      // new AutonIncWristAngle(intake, AutoConstants.SUPPORT_WRIST_ANGLE_TIME),
      // new ParallelCommandGroup(
      // new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_MID_ARM_TIME),
      // new AutonDecArmAngle(armAngle)
      // ),
      // new AutonDecWristAngle(intake, AutoConstants.SCORE_WRIST_ANGLE_TIME),
      // new AutonOuttake(intake, AutoConstants.AUTON_OUTTAKE_TIME)
    );
  }
}
