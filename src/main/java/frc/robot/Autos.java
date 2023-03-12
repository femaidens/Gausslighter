// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;

import com.pathplanner.lib.PathPoint;

/** Add your docs here. */
public class Autos {
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final ArmLateral armLat;
    private final ArmAngle armAng;
    private final ArrayList TrajectoryPointsList;

    public Autos(Drivetrain drivetrain, Intake intake, ArmLateral armLat, ArmAngle armAng, ArrayList TrajectoryPointsList) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.armLat = armLat;
        this.armAng = armAng;
        this.TrajectoryPointsList = TrajectoryPointsList;
    }

    public Command scoreAndCharge(Drivetrain drivetrain){
        return Commands.sequence (
            //drivetrain command to drive forward
            new OpenClaw(intake)
            //drivetrain command to drive backward onto charge
        );
    }

    public Command scoreChargeScore(Drivetrain drivetrain, Intake intake){
        return Commands.sequence(
            //drivetrain command to drive forward
            new OpenClaw(intake),
            //drivetrain command to drive backward onto charge
            new SetArmExtension(Constants.)
        );
    }
}
