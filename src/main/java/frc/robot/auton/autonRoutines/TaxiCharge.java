// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.autonRoutines;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.autonDrivetrain.AutonBalance;
import frc.robot.auton.autonDrivetrain.AutonDrive;
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
      new AutonDrive(drivetrain, -AutoConstants.FIRST_OVERCHARGE_SPEED, 0,0, true, true, AutoConstants.FIRST_OVERCHARGE_TIME),
      new AutonDrive(drivetrain, -AutoConstants.SECOND_OVERCHARGE_SPEED, 0,0, true, true, AutoConstants.SECOND_OVERCHARGE_TIME),

      new WaitCommand(0.5),
      new PrintCommand("finished overcharge")

      // UNNEGATE FOR MATCH
    //   new Charge(drivetrain, -AutoConstants.TAXICHARGE_SPEED, AutoConstants.TAXICHARGE_TIME),
    //   new AutonBalance(drivetrain, AutoConstants.AUTONBALANCE_SPEED, 0, 0, isFinished(), isFinished())
    );
    
  }
}
