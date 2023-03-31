// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.LEDConstants;

public class LED2 extends SubsystemBase {
  /** Creates a new LED2. */
  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuff;
  public LED2() {
    led = new AddressableLED(Ports.LEDPorts.PWM);
    ledBuff = new AddressableLEDBuffer(LEDConstants.LED_PIN_LENGTH);
    led.setLength(LEDConstants.LED_PIN_LENGTH);
    led.setData(ledBuff);
    led.start();
  }

  public void offLEDS(){
    for(int i = 0; i < ledBuff.getLength(); i++){
      ledBuff.setHSV(i, 0, 0, 0);
    }
    led.setData(ledBuff);
  }
  public void altPurpGreen(){
    for(int i = 0; i < ledBuff.getLength(); i++){
      if(i%2 == 1){
        //green light
        ledBuff.setRGB(i, 94, 235, 134);
      }else{
        //purple light
        ledBuff.setRGB(i, 208, 66, 227);
      }
    }
    led.setData(ledBuff);
  }
  public void altGreenPurp(){
    for(int i = 0; i < ledBuff.getLength(); i++){
      if(i%2 == 0){ // <- note the difference
        //green light
        ledBuff.setRGB(i, 94, 235, 134);
      }else{
        //purple light
        ledBuff.setRGB(i, 208, 66, 227);
      }
    }
    led.setData(ledBuff);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
