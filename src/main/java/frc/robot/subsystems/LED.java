// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private Timer timer;
  public LED() {
    timer = new Timer();
    led = new AddressableLED(Ports.LEDPorts.PWM);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }

  public void lightShow(){
    purpGreen();
    showProgramCleanUp();
    new WaitCommand(1.5);
    rainbow();
    showProgramCleanUp();
    new WaitCommand(1.5);
  }
  
  // switches off all LEDs
  public void showProgramCleanUp() {
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++) {
      ledBuffer.setHSV(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  public void rainbow(){
    timer.start();
    if (timer.get() > 5){
      timer.reset();
      return;
    }
    int first = 0; //idk if this is right
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++) {
      final int hue = (first + (i * 180 / LEDConstants.LED_PIN_LENGTH)) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    first += 3;
    // Check bounds
    first %= 180;
  }

  public void purpGreen(){
    timer.start();
    if (timer.get() > 5){
      timer.reset();
      return;
    }
    boolean green = false;
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH/6; i+=1) {
      for (int j = 0; j < 6; j++){
        if (green){
          ledBuffer.setHSV(i*6+j, 96, 255, 88);
        }
        else{
          ledBuffer.setHSV(i*6+j, 203, 195, 227); //purple  
        }
      } //alternating purple n green every 6
      if (i%2 == 0) green = true;
      else{
        green = false;
      }
      led.setData(ledBuffer);
      new WaitCommand(0.5);
    }
  }

  public void CubeLED(){
    timer.start();
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++){
      ledBuffer.setRGB(i, 207, 98, 100); // azure radiance (blue)
    }
    led.setData(ledBuffer);
    if (timer.get() > 2){
        timer.reset();
        return;
    }
  }

  public void ConeLED(){
    timer.start();
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++){
      ledBuffer.setRGB(i, 56, 225, 225); //yellow
    }
    led.setData(ledBuffer);
    if (timer.get() > 2){
      timer.reset();
      return;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
