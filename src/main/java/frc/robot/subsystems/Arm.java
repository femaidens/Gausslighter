// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Ports.*;

public class Arm extends SubsystemBase {
  private final RelativeEncoder extendRetractEncoder; // extending and retraction arm
  private final CANSparkMax angleMotor;
  private final CANSparkMax rightExtendMotor;
  private final CANSparkMax leftExtendMotor;
  private final PIDController anglePIDController; // angle of lifting arm
  private final PIDController extendPIDController;
  private final SimpleMotorFeedforward armExtensionFF;
  private final DutyCycleEncoder angEncoder;
  private final DigitalInput topSwitch;
  private final DigitalInput botSwitch;

  /** Creates a new ExampleSubsystem. */
  public Arm() {
    // motor instantiations
    angleMotor = new CANSparkMax(ArmPorts.ANG_MOTOR_PORT, MotorType.kBrushless);
    angEncoder = new DutyCycleEncoder(ArmPorts.ANG_ENCODER_PORT);
    rightExtendMotor = new CANSparkMax(ArmPorts.RIGHT_EXTEND_MOTOR_PORT, MotorType.kBrushless);
    leftExtendMotor = new CANSparkMax(ArmPorts.LEFT_EXTEND_MOTOR_PORT, MotorType.kBrushless);

    // encoder instantiations
    extendRetractEncoder = leftExtendMotor.getEncoder(); // subject to change

    // feedback controllers
    anglePIDController = new PIDController(ArmConstants.AnglePID.kP,
        ArmConstants.AnglePID.kI, ArmConstants.AnglePID.kD);
    extendPIDController = new PIDController(ArmConstants.ExtendPID.kP,
        ArmConstants.ExtendPID.kI, ArmConstants.ExtendPID.kD);

    // feedforward controllers
    armExtensionFF = new SimpleMotorFeedforward(ArmConstants.FeedForward.kS,
        ArmConstants.FeedForward.kV, ArmConstants.FeedForward.kA);

    // limit switches
    topSwitch = new DigitalInput(ArmPorts.TOP_SWITCH_PORT);
    botSwitch = new DigitalInput(ArmPorts.BOT_SWITCH_PORT);

    // motor config
    leftExtendMotor.setInverted(true);
  }

  public void extendPosition(double position) {
    double extendArmPID = extendPIDController.calculate(extendRetractEncoder.getPosition(), position);
    // double extendArmFF = velocityFF.calculate(vVelocity); just testing pid for
    // now
    leftExtendMotor.setVoltage(extendArmPID); // velocity PID/FF returns voltage

    if (extendArmPID < 0) { // left is extending
      if (topSwitch.get()) { // hit top limit switch (extending)
        leftExtendMotor.set(0);
        rightExtendMotor.set(0);
      } else {
        leftExtendMotor.setVoltage(extendArmPID); // velocity PID/FF returns voltage
        rightExtendMotor.setVoltage(extendArmPID);
      }
    } else {
      if (botSwitch.get()) { // hit bot limit switch (retracting)
        leftExtendMotor.set(0);
        rightExtendMotor.set(0);
      } else {
        leftExtendMotor.setVoltage(extendArmPID); // velocity PID/FF returns voltage
        rightExtendMotor.setVoltage(extendArmPID);
      }
    }

    System.out.println("extend arm pid: " + extendArmPID);
    // rightExtendMotor.setVoltage(extendArmPID); //negate one side
  }

  public void getTicks() {
    // check to see if ticks are inverted (since motor is inverted); if so invert
    // encoder values in constructor
    leftExtendMotor.set(-0.15); // negative = extend
    System.out.println("current ticks:" + extendRetractEncoder.getPosition());
  }

  public void getPositionFactor(double ticks) { // find distance per revolution, return a double after confirmed it
                                                // works
    double currentTicks = extendRetractEncoder.getPosition();
    while (currentTicks < ticks) {
      leftExtendMotor.set(-0.05);
      System.out.println("current ticks: " + currentTicks);
    }
  }

  public void extendDistance(double distance) {
    double currRotations = extendRetractEncoder.getPosition();
    // set the parameter equal to method after test
    // if method works, use getPostionFactor in constructor
    /*
     * while (currTicks < Constants.CPR) {
     * System.out.println("current ticks: " + currTicks);
     * leftExtendMotor.set(-0.05);
     * }
     */
    // if(currRotations != distance) {
    while (currRotations < distance) { // used to add margin
      System.out.println("current rotations: " + currRotations);
      leftExtendMotor.set(-0.3);
    }
    // while(currRotations > distance + extendMargin) {
    // System.out.println("current rotations: " + currRotations);
    // leftExtendMotor.set(0.15);
    // }
    // }
  }

  // arm angle pid not setting angle, change name later
  public void setAngle(double goalAngle) {
    double adjustmentAngle = anglePIDController.calculate(angEncoder.getAbsolutePosition(), goalAngle);
    angleMotor.setVoltage(adjustmentAngle);
  }

  public void stopAngleMotor() {
    angleMotor.set(0);
  }

  public void stopExtensionMotor() {
    leftExtendMotor.set(0);
    // rightExtendMotor.set(0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

// for cansparkmax encoder -> 1 rev = 4096 ticks

// default direction = clockwise