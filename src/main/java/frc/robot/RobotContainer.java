// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.auton.manual.*;
import frc.robot.auton.spbli.*;
import frc.robot.auton.spbli.autonScoreCmds.ScoreHigh;
import frc.robot.auton.spbli.autonScoreCmds.ScoreMid;
import frc.robot.auton.spbli.autonScoreCmds.ScoreMidCharge;
import frc.robot.auton.spbli.autonScoreCmds.ShootHigh;
import frc.robot.auton.spbli.autonScoreCmds.ShootMid;
import frc.robot.auton.spbli.autonScoreCmds.TaxiCharge;
import frc.robot.auton.spbli.autonScoreRoutines.ScoreLongTaxi;
import frc.robot.auton.spbli.autonScoreRoutines.ScoreShortTaxi;
// import frc.robot.autons.Path1;
// import frc.robot.autons.Path2;
// import frc.robot.autons.TestAuton1;
import frc.robot.commands.*;
import frc.robot.commands.Intake2.CloseClaw2;
import frc.robot.commands.Intake2.RunIntake;
import frc.robot.commands.LEDS.ConeLEDS;
import frc.robot.commands.LEDS.CubeLEDS;
import frc.robot.commands.LEDS.PurpGreenLEDS;
import frc.robot.commands.leds.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.intake1.*;
import frc.robot.commands.intake2.*;
import frc.robot.commands.wrist.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    private static final Drivetrain drivetrain = new Drivetrain();
    private final ArmAngle armAngle = new ArmAngle();
    private final ArmLateral armLateral = new ArmLateral();
    private final Intake intake = new Intake();
    private final LED led = new LED();
    private final Limelight limelight = new Limelight();

  // The driver's controller
  CommandXboxController operJoy = new CommandXboxController(Ports.JoystickPorts.OPER_JOY);
  CommandXboxController driveJoy = new CommandXboxController(Ports.JoystickPorts.DRIVE_JOY);
  // private final Joystick lateralJoy = new Joystick(Ports.JoystickPorts.LATERAL_JOY);
  // private final Joystick rotationJoy = new Joystick(Ports.JoystickPorts.ROTATION_JOY);
  private final SendableChooser<Command> autonChooser;
  // autonChooser.addOption("score and charge", new ScoreAndCharge()); adding command to auton chooser, fix syntax idk y it gives errors :(

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSystemDefaults();
    configureButtonBindings();

    // auton config
    autonChooser = new SendableChooser<Command>();
    SmartDashboard.putData("Choose Auto: ", autonChooser);
    autonChooser.setDefaultOption("charge", new Charge(drivetrain, AutoConstants.AUTON_CHARGE_SPEED, AutoConstants.AUTON_CHARGE_TIME)
    );
    
    //shoot
    autonChooser.addOption("shoot mid ", new ShootMid(intake, armAngle, armLateral));
    autonChooser.addOption("shoot high", new ShootHigh(intake, armAngle, armLateral));

    //score
    autonChooser.addOption("score high", new ScoreHigh(intake, armAngle, armLateral));
    autonChooser.addOption("score mid", new ScoreMid(intake, armAngle, armLateral));
    autonChooser.addOption("score mid charge", new ScoreMidCharge(drivetrain, intake, armAngle, armLateral));

    //taxis
    autonChooser.addOption("short taxi", new ShortTaxi(drivetrain));
    autonChooser.setDefaultOption("long taxi", new LongTaxi(drivetrain));
    autonChooser.addOption("score short taxi", new ScoreShortTaxi(drivetrain, intake, armAngle, armLateral));
    autonChooser.addOption("score long taxi", new ScoreLongTaxi(drivetrain, intake, armAngle, armLateral));
    autonChooser.addOption("taxi charge", new TaxiCharge(drivetrain));
    //unused autons

    // autonChooser.addOption("score", new Score(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score left taxi", new ScoreLeftIntake(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score right taxi", new ScoreRightIntake(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("p1", new Path1(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("p2", new Path2(drivetrain));
    //autonChooser.addOption("score and engage", new ScoreAndEngage(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score and drive to game piece", new ScoreDriveToGP(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score, leave community and engage", new ScoreDepartEngage(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score and center intake", new ScoreCenterIntake(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score and left intake", new ScoreLeftIntake(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score and right intake", new ScoreRightIntake(drivetrain, intake, armAngle, armLateral));
    // autonChooser.addOption("score, center intake and engage", new ScoreIntakeEngage(drivetrain, intake, armAngle, armLateral));
    

    // Configure default commands
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
    );

    intake.setDefaultCommand(
      new RunCommand(
        () -> intake.setWristAngleManual(
          MathUtil.applyDeadband(operJoy.getLeftY(), 0.1)),
        intake)
    );
    
    armAngle.setDefaultCommand(
      new RunCommand(
        () -> armAngle.setAngle(
          MathUtil.applyDeadband(operJoy.getRightY(), 0.1)),
        armAngle)
    );

    led.setDefaultCommand(
      new PurpGreenLEDS(led)
    );

  }

  public void configureSystemDefaults(){
    drivetrain.resetGyro();
    drivetrain.resetEncoders();
    // armLateral.retractArm();
    // armAngle.setAngle(PositionConfig.defaultAngle);
    //intake.increaseWristAngle(IntakeConstants.SUPPORT_WRIST_ANGLE);
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
    /* * * DRIVETRAIN * * */
      Trigger xDriveButton = driveJoy.leftBumper();
      xDriveButton
        .whileTrue(new InstantCommand(
          () -> drivetrain.setX(), drivetrain));

      Trigger resetGyroButton = driveJoy.rightBumper();
      resetGyroButton
          .onTrue(new InstantCommand(
            () -> drivetrain.resetGyro(), drivetrain));

      // Trigger slowDriveButton = driveJoy.leftBumper();
      // slowDriveButton
      //     .onTrue(new InstantCommand(
      //       () -> drivetrain.slowSpeed(), drivetrain))
      //     .onFalse(new InstantCommand(
      //       () -> drivetrain.regSpeed(), drivetrain));
      
    /* * * INTAKE 2 * * */
      /* LEDS */
      Trigger coneLEDButton = operJoy.start(); //8 right
      coneLEDButton
        .onTrue(new ConeLEDS(led));
      
      Trigger cubeLEDButton = operJoy.back(); //7 left
      cubeLEDButton
        .onTrue(new CubeLEDS(led));

      /* INTAKE */
      Trigger runIntakeButton = operJoy.x();
      runIntakeButton
        .onTrue(new RunIntake(intake))
        .onFalse(new InstantCommand(
            () -> intake.stopIntakeMotor(), intake));
      
      Trigger reverseIntakeButton = operJoy.y();
      reverseIntakeButton
        .onTrue(new RunCommand(
            () -> intake.reverseIntakeMotor(), intake))
        .onFalse(new InstantCommand(
            () -> intake.stopIntakeMotor(), intake));

      Trigger closeClawButton = operJoy.leftBumper();
      closeClawButton
          .onTrue(new CloseClaw2(intake));

      Trigger openClawButton = operJoy.rightBumper();
      openClawButton
        .onTrue(new OpenClaw(intake))
        .onFalse(new InstantCommand(
          () -> intake.stopIntakeMotor(), intake));

      /* LATERAL */
      Trigger extendButton = operJoy.rightTrigger();
      extendButton
        .onTrue(new RunCommand(
          () -> armLateral.extendArm(), armLateral))
        .onFalse(new RunCommand(
          () -> armLateral.stopExtensionMotors(), armLateral));

      Trigger retractButton = operJoy.leftTrigger();
      retractButton
        .onTrue(new RunCommand(
          () -> armLateral.retractArm(), armLateral))
        .onFalse(new RunCommand(
          () -> armLateral.stopExtensionMotors(), armLateral));
    
    /* * * INTAKE 1 * * */
    /* LEDS */
    // Trigger coneLEDButton = operJoy.y();
    // coneLEDButton
    //     .onTrue(
    //       new StartEndCommand(() -> led.ConeLED(), PurpGreenLEDS(led), led)
    //     );
    // Trigger cubeLEDButton = operJoy.x(); 
    // cubeLEDButton
    //     .onTrue(
    //       new StartEndCommand(() -> led.CubeLED(), () -> led.lightShow(), led)
    //     );
  
    /* INTAKE */
    // Trigger intakeCubeButton = operJoy.leftBumper();
    // intakeCubeButton
    //   .onTrue(new CloseClawCube(intake));
    // Trigger intakeConeButton = operJoy.rightBumper();
    //   intakeConeButton
    //     .onTrue(new CloseClawCone(intake));
    // Trigger scoreButton1 = operJoy.start(); //TESTING NEEDED THIS BUTTON
    // scoreButton1
    //   .onTrue(
    //     new RunCommand(
    //     () -> intake.openClaw(), 
    //     intake)
    //   );
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