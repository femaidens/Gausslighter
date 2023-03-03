// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * Parent class for all autonomous commands
 */
public class AutonBase extends SequentialCommandGroup {
    public Drivetrain drive;
    public static final ProfiledPIDController profiledthetaController =
        new ProfiledPIDController(Constants.AutoConstants.PThetaController, 0, 0,
            Constants.AutoConstants.kThetaControllerConstraints);
    public static final PIDController thetaController =
        new PIDController(Constants.AutoConstants.PThetaController, 0, 0);

    /**
     * Autonomous that aligns limelight then executes a trajectory.
     *
     * @param swerve swerve subsystem
     */
    public AutonBase(Drivetrain drive) {
        this.drive = drive;
        addRequirements(drive);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Creates a SwerveControllerCommand from a Trajectory
     *
     * @param trajectory Trajectory to run
     * @return A SwerveControllerCommand for the robot to move
     */
    public SwerveControllerCommand baseSwerveCommand(Trajectory trajectory) {
        SwerveControllerCommand command = new SwerveControllerCommand(trajectory, drive::getPose,
            Constants.DriveConstants.DRIVE_KINEMATICS,
            new PIDController(Constants.AutoConstants.PXController, 0, 0),
            new PIDController(Constants.AutoConstants.PYController, 0, 0), profiledthetaController,
            drive::setModuleStates, drive);
        return command;
    }

    /**
     * Creates a SwerveController Command using a Path Planner Trajectory
     *
     * @param trajectory a Path Planner Trajectory
     * @return A SwerveControllerCommand for the robot to move
     */
    public PPSwerveControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory,
            drive::getPose, Constants.DriveConstants.DRIVE_KINEMATICS,
            new PIDController(Constants.AutoConstants.PXController, 0, 0),
            new PIDController(Constants.AutoConstants.PYController, 0, 0), thetaController,
            drive::setModuleStates, drive);
        return command;
    }
    // use this
}