// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.spbli.autonRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.auton.spbli.Charge;
import frc.robot.auton.spbli.LongTaxi;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiCharge extends SequentialCommandGroup {
  /** Creates a new TaxiCharge. */
  private final Drivetrain drivetrain;
  public TaxiCharge(Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;

    addCommands(
      new LongTaxi(drivetrain),
      new Charge(drivetrain, -AutoConstants.AUTON_CHARGE_SPEED, AutoConstants.AUTON_TAXICHARGE_TIME)
    );
  }
}
