// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonTaxi;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.autonDrivetrain.AutonDrive;
import frc.robot.subsystems.Drivetrain;

public class ShortTaxi extends SequentialCommandGroup {
  public ShortTaxi(Drivetrain drivetrain) {
    // starting @ left/right, facing gp

    addCommands(
      new InstantCommand(
        () -> drivetrain.resetGyro()
      ),

      new AutonDrive(drivetrain, -AutoConstants.TAXI_SPEED, 0, 0, 
        true, true, AutoConstants.SHORT_TAXI_TIME)); // going same distance as the charge time
 
      //node to gp = 224 => 222 in
  }
}
