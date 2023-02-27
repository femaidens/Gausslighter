package frc.robot.autons;

/** Add your docs here. */
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutonBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * Autonomous that taxis, scores, pushes and scores again.
 */
public class Path2 extends AutonBase {
    Drivetrain drivetrain;
    //Intake intake;
    //Arm arm;

    public Path2(Drivetrain drivetrain) { //add other subsystem parameters once merged
        super(drivetrain);
        //this.intake = intake;
        //this.arm = arm;

        //addRequirements(intake, arm);

        PathPlannerTrajectory p2 = PathPlanner.loadPath("score push score", 4, 3);
        PPSwerveControllerCommand firstCommand = baseSwerveCommand(p2);
        PathPlannerState initialState = p2.getInitialState();

        //score => drive to cone => push to score
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
