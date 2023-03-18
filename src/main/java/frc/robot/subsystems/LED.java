// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  public LED() {
    led = new AddressableLED(Ports.LEDPorts.PWM);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_PIN_LENGTH);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }
  
  // switches off all LEDs
  public void showProgramCleanUp() {
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++) {
      ledBuffer.setHSV(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
  }

  // public void rainbow(){
  //   timer.start();
  //   if (timer.get() > 5){
  //     timer.reset();
  //     return;
  //   }
  //   int first = 0; //idk if this is right
  //   for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++) {
  //     final int hue = (first + (i * 180 / LEDConstants.LED_PIN_LENGTH)) % 180;
  //     ledBuffer.setHSV(i, hue, 255, 128);
  //   }
  //   // Increase by to make the rainbow "move"
  //   first += 3;
  //   // Check bounds
  //   first %= 180;
  // }

  public void purpGreen(){
    boolean green = false;
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH/6; i+=1) {
      for (int j = 0; j < 6; j++){

        if (green){
          ledBuffer.setRGB(i*6+j, 90, 244, 0);
        }
        //i*6 + j
        else{
          ledBuffer.setRGB(i*6+j, 200, 0, 255); //purple  
        }
        //} //alternating purple n green every 6
        // if (i%2 == 0) green = true;
        // else {
        //   green = false;
        // }
        
      }
      green = !green;
      new WaitCommand(3);

    }
    led.setData(ledBuffer);
  }

  public void CubeLED(){
    new PrintCommand("running cube leds");
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++){
      ledBuffer.setRGB(i, 0, 0, 225); // blue
    }
    led.setData(ledBuffer);
  }

  public void ConeLED(){
    new PrintCommand("running cone leds");
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++){
      ledBuffer.setRGB(i,225, 0 ,0); //red
    }
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
