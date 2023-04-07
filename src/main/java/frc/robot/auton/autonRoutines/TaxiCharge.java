// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiCharge extends SequentialCommandGroup {
  /** Creates a new TaxiCharge. */
  public TaxiCharge(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Charge(drivetrain, AutoConstants.OVERCHARGE_SPEED, AutoConstants.OVERCHARGE_TIME),
      //new WaitCommand(0.5),
      new Charge(drivetrain, -AutoConstants.TAXICHARGE_SPEED, AutoConstants.TAXICHARGE_TIME)
    );
  }
}
