// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Ports.*;
import frc.robot.commands.wrist.SetWristAngleVoltage;
import frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final DoubleSolenoid piston1;
  private final DoubleSolenoid piston2;
  private final CANSparkMax wristMotor;
  private final CANSparkMax clawMotor;
  private final PIDController wristPIDController;
  private final SparkMaxAbsoluteEncoder wristEncoder;
  // private double measurement;
  private double setpoint;
  private double autonGoal;
  private boolean isManual = true;

  // private final Timer timer;
  // private static int margin = 3; // margin for claw angle in terms of ticks,
  // can change later

  public Intake() {

    // piston instantiations
    piston1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakePorts.PISTON1_FORWARD_PORT,
        IntakePorts.PISTON1_REVERSE_PORT);
    piston2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakePorts.PISTON2_FORWARD_PORT,
        IntakePorts.PISTON2_REVERSE_PORT);

    // motor instantiations
    wristMotor = new CANSparkMax(IntakePorts.WRIST_MOTOR_PORT, MotorType.kBrushless);
    clawMotor = new CANSparkMax(IntakePorts.CLAW_MOTOR_PORT, MotorType.kBrushless);
   
    // encoder instantiations
    wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    setpoint = wristEncoder.getPosition();

    // motor configs
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setInverted(false);
    wristPIDController = new PIDController(IntakeConstants.wristkP, IntakeConstants.wristkI, IntakeConstants.wristkD);

    // current limits
    wristMotor.setSmartCurrentLimit(IntakeConstants.WRIST_MOTOR_CURRENT_LIMIT);
    wristMotor.setSecondaryCurrentLimit(IntakeConstants.WRIST_MOTOR_SECONDARY_LIMIT);
    clawMotor.setSmartCurrentLimit(IntakeConstants.CLAW_MOTOR_CURRENT_LIMIT);
    clawMotor.setSecondaryCurrentLimit(IntakeConstants.CLAW_MOTOR_SECONDARY_LIMIT);

    // encoder config
    wristEncoder.setPositionConversionFactor(360);

    // timer = new Timer();
  }

  // MANUAL
  public void setWristAngleManual(double input){
    isManual = true;
    double wristAngleSpeed = 0.2;
    double currentWristAngle = getWristAngle();

    if(currentWristAngle < IntakeConstants.DEFAULT_WRIST_ANGLE || currentWristAngle > 300){
      setpoint = IntakeConstants.DEFAULT_WRIST_ANGLE;
      if(currentWristAngle > 300){
        double measurement = Math.abs(360 - wristEncoder.getPosition()) + IntakeConstants.DEFAULT_WRIST_ANGLE;
        wristMotor.setVoltage(-calculateVoltage(measurement));
      }
      else{
        setAngleVoltage();
      }
      System.out.println("at default wrist limit");
    }

    else if (currentWristAngle > IntakeConstants.SUPPORT_WRIST_ANGLE && currentWristAngle < 240){
      setpoint = IntakeConstants.SUPPORT_WRIST_ANGLE;
      System.out.println("at support wrist limit");
    }

    else{
      if(input < 0){ // neg input pushing up
        wristMotor.set(wristAngleSpeed);
        setpoint = wristEncoder.getPosition();
      }
      else if (input > 0 ){ // pos input pushing down
        wristMotor.set(-wristAngleSpeed); //increase
        setpoint = wristEncoder.getPosition();
      }
      else {
        // stopWristMotor();
        setAngleVoltage();
      }
    }
  }

  public void setWristAngle(double goalAngle){
    double currentAngle = wristEncoder.getPosition();
    if (goalAngle > currentAngle){
      increaseWristAngle(goalAngle);
    }
    else if (goalAngle < currentAngle){
      decreaseWristAngle(goalAngle);
    }
  }

  public void closeClaw() { // retracting pistons
    piston1.set(Value.kForward);
    // piston2.set(Value.kForward);
    //System.out.println("piston state: " + piston1.);
  }

  public void openClaw() { // extends pistons & clamps onto the gamepiece
    piston1.set(Value.kReverse);
    // System.out.println("left piston out");
  }

  public void closeClawCube() { // extends pistons & clamps onto the gamepiece
    piston1.set(Value.kReverse);
    piston2.set(Value.kReverse);
    // System.out.println("both pistons out");
  }
  
  public void closeClawCone() { // extends pistons & clamps onto the gamepiece
    piston1.set(Value.kReverse);
    // System.out.println("left piston out");
  }

  public void atWristLimit(){
    if(wristEncoder.getPosition() <= IntakeConstants.DEFAULT_WRIST_ANGLE || wristEncoder.getPosition() >= IntakeConstants.SUPPORT_WRIST_ANGLE){
      System.out.println("at wrist limit");
      stopWristMotor();
    }
  }

  public void setAngleVoltage(){
    double wristAngleVoltage = wristPIDController.calculate(getWristAngle(), setpoint);
    wristMotor.setVoltage(wristAngleVoltage);
    System.out.println("Running Wrist PID");
  }

  public double calculateVoltage(double measurement){
    return wristPIDController.calculate(measurement, setpoint);
  }

  public void setAutonWristAngle(double autonSetpoint){
    autonGoal = autonSetpoint;
    double autonWristAngleVoltage = wristPIDController.calculate(getWristAngle(), autonSetpoint);
    wristMotor.setVoltage(autonWristAngleVoltage);
  }

  public void setSingleIntakeAngle(){
    isManual = false;
    //System.out.println("running single pid");
    setpoint = IntakeConstants.INTAKE_SINGLE_WRIST_ANGLE;
  }

  public void setDoubleIntakeAngle(){ //also works for shooting..?
    isManual = false;
    //System.out.println("running double pid");
    setpoint = IntakeConstants.INTAKE_DOUBLE_WRIST_ANGLE;
  }

  public void updateTeleopSetpoint(){
    setpoint = autonGoal;
  }

  public boolean getIsManual(){
    return isManual;
  }

  public boolean atWristAngle(double angle){
    double currentAngle = wristEncoder.getPosition();
    if (Math.abs(currentAngle - angle) < 2){
        return true;
    }
    return false; 
  }

  public double getWristAngle(){
    return wristEncoder.getPosition();
  }

  // originally setDefaultWristAngle
  public void decreaseWristAngle(double angle){
    double currentAngle = wristEncoder.getPosition();
    setpoint = currentAngle;
    if (Math.abs(currentAngle - angle) < 1){ // test angle of wrist when fully down
        wristMotor.stopMotor();
    }
    System.out.println("decreasing wrist ang");
    wristMotor.set(-0.2);
  }

  public void increaseWristAngle(double angle){
    double currentAngle = wristEncoder.getPosition();
    setpoint = currentAngle;
    if (Math.abs(currentAngle - angle) < 1){ // test angle of wrist when fully down
        wristMotor.stopMotor();
    }
    System.out.println("increasing wrist ang");
    wristMotor.set(0.2);
  }

  public void runIntakeMotor(){
    clawMotor.set(0.8);
  }

  public void reverseIntakeMotor(){
    System.out.println("running claw wheels");
    clawMotor.set(-0.8);
  }

  public void stopIntakeMotor(){
    clawMotor.set(0);
  }

  public void runWristMotor(double speed){
    wristMotor.set(speed);
  }

  public void stopWristMotor() {
    wristMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // boolean box
    SmartDashboard.putBoolean("@ double wrist angle", atWristAngle(IntakeConstants.INTAKE_DOUBLE_WRIST_ANGLE));
    SmartDashboard.putBoolean("@ single wrist angle", atWristAngle(IntakeConstants.INTAKE_SINGLE_WRIST_ANGLE));

    SmartDashboard.putBoolean("@ default wrist angle", atWristAngle(IntakeConstants.DEFAULT_WRIST_ANGLE));
    SmartDashboard.putBoolean("@ score wrist angle", atWristAngle(IntakeConstants.SCORE_WRIST_ANGLE));

    // values
    SmartDashboard.putNumber("curr. wrist angle", wristEncoder.getPosition()); 
    SmartDashboard.putNumber("desired wrist angle: ", setpoint);
    SmartDashboard.putNumber("auton setpoint: ", autonGoal);
    // SmartDashboard.putNumber("curr. wrist speed ", wristMotor.get());

    //if(!isManual){
      // double wristAngleVoltage = wristAnglePID.calculate(wristEncoder.getPosition(), setpoint); 
      // wristMotor.setVoltage(wristAngleVoltage);
    //}
  }
}