// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class Gyro extends SubsystemBase {
  public final AHRS gyro;
  /** Creates a new Gyro. */
  public Gyro() {
    configGyro();
    gyro = new AHRS();
  }
  public double getHeading(){
    return gyro.getAngle();
  }
  public void configGyro(){
    gyro.reset();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("angle", gyro.getAngle());
    System.out.println(gyro.getAngle());
    System.out.println("Raw axis: " + gyro.getRawGyroZ());
  }
}
