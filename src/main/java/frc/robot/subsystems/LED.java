// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  public LED() {
    led = new AddressableLED(Ports.LEDPorts.PWM);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    led.start();
  }

  // switches off all LEDs
  public void showProgramCleanUp(int delayTime) {
    for (int i = 0; i < Constants.LEDConstants.LED_PIN_LENGTH; i++) {
      ledBuffer.setHSV(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
 }

 
  public void lightShow(){

  }

  public void CubeLED(){
    for (int i = 0; i < Constants.LEDConstants.LED_PIN_LENGTH; i++){
      ledBuffer.setRGB(i, 139, 0, 139);
    }
    led.setData(ledBuffer);
  }

  public void ConeLED(){
    for (int i = 0; i < Constants.LEDConstants.LED_PIN_LENGTH; i++){
      ledBuffer.setRGB(i, 255, 0, 255);
    }
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    CubeLED();
    led.setData(ledBuffer);
  }
}
