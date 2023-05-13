// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake2;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.wrist.SetWristAngleVoltage;
import frc.robot.commands.wrist.SingleWristAngle;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SingleIntakeRoutine extends SequentialCommandGroup {
  /** Creates a new SingleIntakeRoutine. */
  public SingleIntakeRoutine(Intake intake, ArmLateral armLateral, ArmAngle armAngle) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        //new AutonSetArmLength(armLateral, PositionConfig.highLength),
        new InstantCommand(
          () -> armAngle.setSingleArmAngle(), armAngle),
        //new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME), //time to extend to single ss
        new InstantCommand(
          () -> intake.setSingleIntakeAngle(), intake)
        // new PrintCommand("running single intake routine")
      )
      // new ParallelCommandGroup(
      //   //new AutonSetArmLength(armLateral, PositionConfig.highLength),
      //   new InstantCommand(
      //     () -> armAngle.setSingleArmAngle(), armAngle),
      //   //new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME), //time to extend to single ss
      //   new SingleWristAngle(intake)
      //   // new PrintCommand("running single intake routine")
      // )


      // new ParallelCommandGroup(
      //   //new AutonSetArmLength(armLateral, PositionConfig.highLength),
      //   //new InstantCommand(
      //   //  () -> armAngle.setSingleArmAngle(), armAngle),
      //   //new AutonExtendArm(armLateral, AutoConstants.AUTON_EXTEND_HIGH_ARM_TIME), //time to extend to single ss
      //   new RunCommand(
      //     () -> intake.setWristAngle(IntakeConstants.INTAKE_SINGLE_WRIST_ANGLE), intake),
      //   new PrintCommand("running single intake routine")
      // )
    );
  }
}
