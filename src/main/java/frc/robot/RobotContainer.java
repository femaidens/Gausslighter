// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.swing.text.Position;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.Constants.ArmConstants.PositionConfig;
import frc.robot.Ports.ButtonPorts;
import frc.robot.autons.Path1;
import frc.robot.autons.Path2;
import frc.robot.autons.TestAuton1;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import java.util.List;
// import java.util.HashMap;
// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();

  // The driver's controller
  XboxController operJoy = new XboxController(Ports.JoystickPorts.OPER_JOY);
  // private final Joystick lateralJoy = new Joystick(Ports.JoystickPorts.LATERAL_JOY);
  // private final Joystick rotationJoy = new Joystick(Ports.JoystickPorts.ROTATION_JOY);
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.resetGyro();
    drivetrain.resetEncoders();
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    autonChooser.addOption("p1", new Path1(drivetrain, intake, arm));
    autonChooser.addOption("p2", new Path2(drivetrain));

    // Configure default commands
    drivetrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrain.drive( // all joy.get values were prev negative
                MathUtil.applyDeadband(operJoy.getRightY(), 0.1),
                MathUtil.applyDeadband(operJoy.getRightX(), 0.1),
                MathUtil.applyDeadband(operJoy.getLeftX(), 0.1),
                true, true),
            drivetrain)

        // new RunCommand(
        //     () -> drivetrain.drive(
        //         MathUtil.applyDeadband(-lateralJoy.getY(), 0.05),
        //         MathUtil.applyDeadband(-lateralJoy.getX(), 0.05),
        //         MathUtil.applyDeadband(-rotationJoy.getX(), 0.05),
        //         true),
        //     drivetrain)
    );
  }

  public void configureAuton() {

    SmartDashboard.putData("Choose Auto: ", autonChooser);
    autonChooser.addOption("p1", new Path1(drivetrain, intake, arm));
    autonChooser.addOption("p2", new Path2(drivetrain));
    autonChooser.addOption("test auton", new TestAuton1(drivetrain, intake, arm));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(operJoy, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(
            () -> drivetrain.setX(),
            drivetrain));

    new JoystickButton(operJoy, 6) // right button
    .whileTrue(new RunCommand(
        () -> drivetrain.resetGyro(),
        drivetrain));
    
    // figure out better/more efficient way of creating/binding these cmds to buttons
    final Trigger midCubeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.A);
    midCubeButton.onTrue(Commands.sequence(new SetArmAngle(arm, PositionConfig.midCubeAngle), 
      new SetArmExtension(arm, PositionConfig.midCubeExtend), new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger midConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.B);
    midConeButton.onTrue(Commands.sequence(new SetArmAngle(arm, PositionConfig.midConeAngle), 
      new SetArmExtension(arm, PositionConfig.midConeExtend), new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger highCubeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.X); //change command for testing angle
    highCubeButton.onTrue(Commands.sequence(new SetArmAngle(arm, PositionConfig.highCubeAngle), 
      new SetArmExtension(arm, PositionConfig.highCubeExtend), new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger highConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.Y);
    highConeButton.onTrue(Commands.sequence(new SetArmAngle(arm, PositionConfig.highConeAngle), 
      new SetArmExtension(arm, PositionConfig.highConeExtend), new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger resetIntakeButton = new JoystickButton(operJoy, ButtonPorts.RESET_INTAKE_BUTTON_PORT);
    resetIntakeButton.onTrue(Commands.parallel(new SetArmAngle(arm, ArmConstants.defaultArmAngle), 
      new SetArmExtension(arm, PositionConfig.defaultExtension), new SetClawAngle(intake, IntakeConstants.defaultClawAngle)));

    final Trigger floorScoreButton = new JoystickButton(operJoy, ButtonPorts.FLOOR_SCORE_BUTTON_PORT);
    floorScoreButton.onTrue(Commands.sequence(new OpenClaw(intake), 
      new SetArmExtension(arm, PositionConfig.defaultExtension), new SetClawAngle(intake, IntakeConstants.clawAngle)));

    final Trigger floorIntakeButton = new JoystickButton(operJoy, ButtonPorts.FLOOR_INTAKE_BUTTON_PORT);
    floorIntakeButton.onTrue(Commands.sequence(new OpenClaw(intake), new IntakeGP(intake), new CloseClaw(intake)));

    final Trigger humanPlayerButton = new JoystickButton(operJoy, ButtonPorts.HP_BUTTON_PORT);
    humanPlayerButton.onTrue(Commands.sequence(new SetArmAngle(arm, PositionConfig.highConeAngle), 
      new SetArmExtension(arm, PositionConfig.midConeExtend), new SetClawAngle(intake, IntakeConstants.clawAngle)));
    // substation distance (95cm) is similar to mid node distance (90cm)
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.AUTON_MAX_SPEED,
    //     AutoConstants.AUTON_MAX_ACC)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.DRIVE_KINEMATICS);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.PThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     drivetrain::getPose, // Functional interface to feed supplier
    //     DriveConstants.DRIVE_KINEMATICS,

    //     // Position controllers
    //     new PIDController(AutoConstants.PXController, 0, 0),
    //     new PIDController(AutoConstants.PYController, 0, 0),
    //     thetaController,
    //     drivetrain::setModuleStates,
    //     drivetrain);

    // // Reset odometry to the starting pose of the trajectory.
    // drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
    return autonChooser.getSelected();
  }
}