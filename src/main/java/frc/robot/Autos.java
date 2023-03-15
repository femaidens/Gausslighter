// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/** Add your docs here. */
public class Autos {
    private final Drivetrain drivetrain;
    private final Intake intake;
    private final ArmLateral armLat;
    private final ArmAngle armAng;
    private final ArrayList TrajectoryPointsList;
    private final SendableChooser<Command> chooser;
    private final SwerveDriveKinematics kinematics;
    private final PIDController xPIDController;
    private final PIDController yPIDController;
    private final PIDController rotPIDController;
    private final SwerveModuleState states;
    private List<PathPoint> points;
    private Object SCORE_AND_CHARGE;

    public Autos(Drivetrain drivetrain, Intake intake, ArmLateral armLat, 
    ArmAngle armAng, ArrayList TrajectoryPointsList, SwerveDriveKinematics kinematics) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.armLat = armLat;
        this.armAng = armAng;
        this.kinematics = kinematics;
        this.TrajectoryPointsList = TrajectoryPointsList;
        chooser = new SendableChooser<>();
        chooser.addOption("score and charge", ());

        var states =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))};
    }        

    public Command scoreAndCharge(Drivetrain drivetrain, ArmLateral armLat, ArmAngle armAng){
        return Commands.sequence (
            new SetArmAngle(armAng, ArmConstants.PositionConfig.highConeAngle, true),
            new SetArmExtension(armLat, ArmConstants.PositionConfig.highLength),
            new OpenClaw(intake),
            new Follow(drivetrain, SCORE_AND_CHARGE, true)
        );
    }

    public Command scoreChargeScore(Drivetrain drivetrain, Intake intake){
        return Commands.sequence(
            //drivetrain command to drive forward
            new SetArmAngle(armAng, ArmConstants.PositionConfig.highConeAngle, true),
            new SetArmExtension(armLat, ArmConstants.PositionConfig.highLength),
            new OpenClaw(intake)
            //drivetrain command to drive backward onto charge
        );
    }

}
