// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.autonDrivetrain.AutonBalance;
import frc.robot.auton.autonDrivetrain.AutonDrive;
// import frc.robot.subsystems.ArmAngle;
// import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Intake;

public class Charge extends SequentialCommandGroup {
  public Charge(Drivetrain drivetrain, double chargeSpeed, double chargeTime) {
    // SET UP ROBOT TO FACE CHARGE UP
    // starting on the center position; right in front and facing charge up
    addCommands(
      new InstantCommand(
        () -> drivetrain.resetGyro()
      ),
      new PrintCommand("Running charge"),

      new AutonDrive(drivetrain, -chargeSpeed, 0, 0, true, true, chargeTime),
      new AutonBalance(drivetrain, -AutoConstants.AUTONBALANCE_SPEED, 0, 0, true, true)

      //0.2 speed, 1 second = 37 inches
    );
  }
}