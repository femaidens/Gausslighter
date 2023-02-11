// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double wristSpeed = 0.1;
  public static final int CPR = 4096;

  public static class PIDConstants {
    public static final double Kp = 0.2;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;
    public double left_speed, right_speed;
    static double min_error = 0.1; //sets an error deadband/ minimum value
    static double min_command = 0.0;
    static double current_error = 0; 
    static double previous_error = 0;
    static double integral = 0;
    static double derivative = 0;
    static double adjust = 0;
    static double time = 0.1; 
  }
}
