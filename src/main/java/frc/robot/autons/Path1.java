// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

/** Add your docs here. */
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutonBase;
import frc.robot.subsystems.Drivetrain;

/**
 * Autonomous that taxis, scores, and charges.
 */
public class Path1 extends AutonBase {
    Drivetrain drivetrain;
    //Intake intake;
    //Arm arm;

    public Path1(Drivetrain drivetrain) { //add other subsystem parameters once merged
        super(drivetrain);
        //this.arm = arm;
        //addRequirements(arm);
        PathPlannerTrajectory p1 = PathPlanner.loadPath("score and charge", 4, 3);
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(p1);
        PathPlannerState initialState = p1.getInitialState();

        //score => drive => charge
        addCommands(new InstantCommand(() -> drivetrain.resetGyro()),
            new InstantCommand(
                () -> drivetrain.resetOdometry(new Pose2d(initialState.poseMeters.getTranslation(),
                    initialState.holonomicRotation))),
            firstCommand);

    }

    public void end(){
        //end commands here
    }
}