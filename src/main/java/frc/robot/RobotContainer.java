// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.*;
import frc.robot.Ports.ButtonPorts;
import frc.robot.Ports.XboxControllerMap.Button;
import frc.robot.auton.ScoreAndCharge;
// import frc.robot.autons.Path1;
// import frc.robot.autons.Path2;
// import frc.robot.autons.TestAuton1;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmAngle;
import frc.robot.subsystems.ArmLateral;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private final ArmAngle armAngle = new ArmAngle();
    private final ArmLateral armLateral = new ArmLateral();
    private final Intake intake = new Intake();

  // The driver's controller
  XboxController operJoy = new XboxController(Ports.JoystickPorts.OPER_JOY);
  XboxController driveJoy = new XboxController(Ports.JoystickPorts.DRIVE_JOY);
  // private final Joystick lateralJoy = new Joystick(Ports.JoystickPorts.LATERAL_JOY);
  // private final Joystick rotationJoy = new Joystick(Ports.JoystickPorts.ROTATION_JOY);
  private final SendableChooser<Command> autonChooser;
  // autonChooser.addOption("score and charge", new ScoreAndCharge()); adding command to auton chooser, fix syntax idk y it gives errors :(

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // drivetrain.resetGyro();
    // drivetrain.resetEncoders();

    // // auton config
    // SmartDashboard.putData("Choose Auto: ", autonChooser);
    // autonChooser.addOption("p1", new Path1(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("p2", new Path2(drivetrain));

    // Configure default commands

    armLateral.setDefaultCommand(
      new RunCommand(
        () -> armLateral.setLength(
          MathUtil.applyDeadband(operJoy.getLeftY(), 0.1)),
        armLateral)
    );

    drivetrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrain.drive( // all joy.get values were prev negative
                MathUtil.applyDeadband(-driveJoy.getRightY(), 0.1),
                MathUtil.applyDeadband(-driveJoy.getRightX(), 0.1),
                MathUtil.applyDeadband(-driveJoy.getLeftX(), 0.1),
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

    autonChooser = new SendableChooser<Command>();

    autonChooser.addOption("score and charge", new ScoreAndCharge(drivetrain, intake, armAngle, armLateral));
    autonChooser.addOption("score and intake", new ScoreAndCharge(drivetrain, intake, armAngle, armLateral));
    autonChooser.addOption("score, leave community, and charge", new ScoreAndCharge(drivetrain, intake, armAngle, armLateral));
    autonChooser.addOption("score and drive to game piece", new ScoreAndCharge(drivetrain, intake, armAngle, armLateral));
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
    //changing claw INTAKE 1
    // new JoystickButton(operJoy, Button.A)
    //     .onTrue(new OpenClaw(intake));
    // new JoystickButton(operJoy, Button.B)
    //     .onTrue(new CloseClawCube(intake)); (make commands for these)
    // new JoystickButton(operJoy, Button.Y)
    //     .onTrue(new CloseClawCone(intake));
    //changing claw INTAKE 2
    new JoystickButton(operJoy, Button.A)
        .onTrue(new OpenClaw(intake));
    new JoystickButton(operJoy, Button.B)
        .onTrue(new CloseClaw2(intake));

    //changing arm angles (trigger buttons)
    new JoystickButton(operJoy, Button.LT)
        .onTrue(
          new RunCommand(
            () -> armAngle.decreaseAngle(operJoy.getLeftTriggerAxis()),
            armAngle));
    new JoystickButton(operJoy, Button.RT)
        .onTrue(
          new RunCommand(
            () -> armAngle.increaseAngle(operJoy.getRightTriggerAxis()),
            armAngle));
    // new JoystickButton(driveJoy, XboxController.Button.kA.value)
    //     .whileTrue(
    //       new RunCommand(
    //         () -> drivetrain.setX(),
    //         drivetrain));

    // // resets robot heading (gyro)
    // new JoystickButton(driveJoy, 6) // RB
    //     .onTrue(
    //         new RunCommand(
    //           () -> drivetrain.resetGyro(),
    //           drivetrain));
    
    // figure out better/more efficient way of creating/binding these cmds to buttons
    // final Trigger midCubeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.A);
    // midCubeButton.onTrue(Commands.sequence(
    //   new SetArmAngle(armAngle, PositionConfig.midCubeAngle), 
    //   new SetArmExtension(armLateral, PositionConfig.midCubeExtend), 
    //   new SetClawAngle(intake, IntakeConstants.clawAngle)));

    // final Trigger midConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.B);
    // midConeButton.onTrue(Commands.sequence(
    //   new SetArmAngle(armAngle, PositionConfig.midConeAngle)));
    //   // new SetArmExtension(armLateral, PositionConfig.midConeExtend), 
    //   // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    /* 
    final JoystickButton midCubeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.A);
    midCubeButton.whileTrue(
      // new RunCommand(
      //   () -> armAngle.setAngle(armAngle.getArmAngle(), ArmConstants.PositionConfig.midConeAngle))
      // );

       //Commands.sequence
      new RunCommand(
        () -> armAngle.downArm(),
        armAngle));
      // new SetArmAngle(armAngle, PositionConfig.midCubeAngle));
      // new SetArmExtension(armLateral, PositionConfig.midCubeExtend), 
      // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    */

    /*
    final Trigger midConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.B);
    midConeButton.onTrue(
      new RunCommand(
        () -> intake.closeClawCone(),
        intake));

      // new SetArmAngle(armAngle, PositionConfig.midConeAngle)));
      // new SetArmExtension(armLateral, PositionConfig.midConeExtend), 
      // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    */

    // final Trigger highCubeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.X); //change command for testing angle
    // highCubeButton.onTrue(Commands.sequence(
    //   new SetArmAngle(armAngle, PositionConfig.highCubeAngle)));
    //   // new SetArmExtension(armLateral, PositionConfig.highCubeExtend), 
    //   // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    // final Trigger highConeButton = new JoystickButton(operJoy, Ports.XboxControllerMap.Button.Y);
    // highConeButton.onTrue(Commands.sequence(
    //   new SetArmAngle(armAngle, PositionConfig.highConeAngle)));
    //   // new SetArmExtension(armLateral, PositionConfig.highConeExtend), 
    //   // new SetClawAngle(intake, IntakeConstants.clawAngle)));

    // final Trigger resetIntakeButton = new JoystickButton(operJoy, ButtonPorts.RESET_INTAKE_BUTTON_PORT);
    // resetIntakeButton.onTrue(
    //   // Commands.parallel(
    //   new SetArmAngle(armAngle, ArmConstants.DEFAULT_ARM_ANGLE));
    //   // new SetArmExtension(armLateral, PositionConfig.defaultExtension), 
    //   // new SetClawAngle(intake, IntakeConstants.defaultClawAngle)));

    // final Trigger floorScoreButton = new JoystickButton(operJoy, ButtonPorts.FLOOR_SCORE_BUTTON_PORT);
    // floorScoreButton.onTrue(Commands.sequence(
    //   new OpenClaw(intake), 
    //   new SetArmExtension(armLateral, PositionConfig.defaultExtension), 
    //   new SetClawAngle(intake, IntakeConstants.clawAngle)));

    // final Trigger floorIntakeButton = new JoystickButton(operJoy, ButtonPorts.FLOOR_INTAKE_BUTTON_PORT);
    // floorIntakeButton.onTrue(Commands.sequence(
    //   new OpenClaw(intake), 
    //   new IntakeGP(intake), 
    //   new CloseClaw(intake)));

    // final Trigger humanPlayerButton = new JoystickButton(operJoy, ButtonPorts.HP_BUTTON_PORT);
    // humanPlayerButton.onTrue(Commands.sequence(
    //   new SetArmAngle(armAngle, PositionConfig.highConeAngle))); 
    //   // new SetArmExtension(armLateral, PositionConfig.midConeExtend), 
    //   // new SetClawAngle(intake, IntakeConstants.clawAngle)));
    // // substation distance (95cm) is similar to mid node distance (90cm)
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

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     drivetrain::getPose, // Functional interface to feed supplier
    //     DriveConstants.DRIVE_KINEMATICS,


    // // Reset odometry to the starting pose of the trajectory.
    // drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false));
    return autonChooser.getSelected();
  }
}