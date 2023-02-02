// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.Autos;
import frc.robot.commands.ConeExtend;
import frc.robot.commands.ConeRetract;
import frc.robot.commands.CubeExtend;
import frc.robot.commands.Retract;
import frc.robot.subsystems.TestSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final DriveTrainSubsystem m_DriveTrain = new DriveTrainSubsystem();
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final TestSolenoid testSolenoid = new TestSolenoid();
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_DriveTrain::exampleCondition)
    //     .onTrue(new DriveCommand(m_DriveTrain));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_DriveTrain.exampleMethodCommand());
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem)); 
    new JoystickButton(m_driverController, Ports.XboxControllerMap.Button.X)
      .onTrue(new ConeExtend(testSolenoid));
    //new JoystickButton(m_driverController, Ports.XboxControllerMap.Button.Y)
    //  .onTrue(new ConeRetract(testSolenoid));
    new JoystickButton(m_driverController, Ports.XboxControllerMap.Button.A)
      .onTrue(new CubeExtend(testSolenoid));
    new JoystickButton(m_driverController, Ports.XboxControllerMap.Button.B)
      .onTrue(new Retract(testSolenoid));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   //return Autos.exampleAuto(m_DriveTrain);
  // }
}
