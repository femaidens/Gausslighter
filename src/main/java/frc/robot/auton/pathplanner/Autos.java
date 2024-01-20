// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton.pathplanner;

//import com.pathplanner.lib.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Autos {
    
    private final Drivetrain drivetrain;

    public Autos (Drivetrain drivetrain) {

        this.drivetrain = drivetrain;
    }

    // PathPlannerTrajectory scoreCharge= PathPlanner.loadPath("score and charge", new PathConstraints(AutoConstants.AUTON_MAX_SPEED, AutoConstants.AUTON_MAX_ACC));
    // PathPlannerTrajectory centerScoreTwoEngage = PathPlanner.loadPath("center score two and engage", new PathConstraints(AutoConstants.AUTON_MAX_SPEED, AutoConstants.AUTON_MAX_ACC));

    // public static final class IntakePaths {
    //     PathPlannerTrajectory scoreLeftIntake = PathPlanner.loadPath("score and left intake", new PathConstraints(AutoConstants.AUTON_MAX_SPEED, AutoConstants.AUTON_MAX_ACC));
    //     PathPlannerTrajectory scoreRightIntake = PathPlanner.loadPath("score and right intake", new PathConstraints(AutoConstants.AUTON_MAX_SPEED, AutoConstants.AUTON_MAX_ACC));
    // }
    // public static final class ScoreMultiplePaths {
    //     PathPlannerTrajectory leftScoreTwo = PathPlanner.loadPath("left score two", new PathConstraints(AutoConstants.AUTON_MAX_SPEED, AutoConstants.AUTON_MAX_ACC));
    //     PathPlannerTrajectory rightScoreTwo = PathPlanner.loadPath("right score two", new PathConstraints(AutoConstants.AUTON_MAX_SPEED, AutoConstants.AUTON_MAX_ACC));
    //     PathPlannerTrajectory centerScoreTwo = PathPlanner.loadPath("center score two", new PathConstraints(AutoConstants.AUTON_MAX_SPEED, AutoConstants.AUTON_MAX_ACC));
    // }

//     public static CommandBase testAuto(Drivetrain drivetrain){
//         HashMap<String, Command> eventMap = new HashMap<>();
//         eventMap.put("marker1", new PrintCommand("Passed marker 1"));
  
//         // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
//         // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
//         SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
//             drivetrain::getPose,
//   // Pose2d supplier
//             drivetrain::resetOdometry,
//   // Pose2d consumer, used to reset odometry at the beginning of auto
//             new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
//   // PID constants to correct for translation error (used to create the X and Y PID controllers)
//             new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
//   // PID constants to correct for rotation error (used to create the rotation controller)
//             drivetrain::setChassisSpeeds,
//   // Module states consumer used to output to the drive subsystem
//             eventMap,
//             false,
//   // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
//             drivetrain
//   // The drive subsystem. Used to properly set the requirements of path following commands
//         );
//         return Commands.sequence(autoBuilder.fullAuto());
//       }
//   //    swerve.postTrajectory(example);
//       return Commands.sequence(new FollowTrajectory(drivetrain, leftScore, true));
//     }
  
    }




