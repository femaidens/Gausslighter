// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.spbli.autonScoreCmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.spbli.AutonOuttake;
import frc.robot.auton.spbli.autonArmCmds.AutonDecArmAngle;
import frc.robot.auton.spbli.autonArmCmds.AutonExtendArm;
import frc.robot.auton.spbli.autonWrist.AutonDecWristAngle;
import frc.robot.auton.spbli.autonWrist.AutonIncWristAngle;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHigh extends SequentialCommandGroup {
  /** Creates a new ShootMid. */
  public ShootHigh(Intake intake, ArmAngle armAngle, ArmLateral armLateral) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AutonIncWristAngle(intake, AutoConstants.SUPPORT_WRIST_ANGLE_TIME),
        new ParallelCommandGroup(
        new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME),
        new AutonDecArmAngle(armAngle)
        ),
        new AutonDecWristAngle(intake, AutoConstants.SCORE_WRIST_ANGLE_TIME),
        new AutonOuttake(intake, AutoConstants.AUTON_OUTTAKE_TIME)
    );
  }
}
