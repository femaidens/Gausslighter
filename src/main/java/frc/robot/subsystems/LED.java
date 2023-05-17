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
import frc.robot.Constants.*;
import frc.robot.Ports;


public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  // private final Timer timer;
  private boolean purple;
  private int time;
  public LED() {
    led = new AddressableLED(Ports.LEDPorts.PWM);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_PIN_LENGTH);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    // timer = new Timer();
    purple = true;
    time = 0;
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
      ledBuffer.setRGB(i, 0, 0, 255); // blue
    }
    led.setData(ledBuffer);
  }

  public void ConeLED(){
    new PrintCommand("running cone leds");
    for (int i = 0; i < LEDConstants.LED_PIN_LENGTH; i++){
      ledBuffer.setRGB(i,255, 255 ,0); //red
    }
    led.setData(ledBuffer);
  }

  public void altPurpGreen(){
    for(int i = 0; i < ledBuffer.getLength(); i++){
      if(i%2 == 1){
        //better green light
        ledBuffer.setRGB(i, 0, 255, 0);
      }else{
        //better purple light
        ledBuffer.setRGB(i, 185, 0, 255);
      }
    }
    led.setData(ledBuffer);
  }

  public void altGreenPurp(){
    for(int i = 0; i < ledBuffer.getLength(); i++){
      if(i%2 == 0) { // <- note the difference
        //green light
        ledBuffer.setRGB(i, 0, 255, 0);
      }
      else {
        //purple light
        ledBuffer.setRGB(i, 185, 0, 255);
      }
    }
    led.setData(ledBuffer);
  }

  public void lightShow(Timer timer){
    System.out.println("running lightshow");
    if(timer.get()%0.5 < 0.25){
      System.out.println("fishy");
      altPurpGreen();
    } else {
      System.out.println("not fishy");
      altGreenPurp();
    }
  }

  public void simplifiedLightShow(){
    System.out.println("running simp lights");
    if(purple){
      altPurpGreen();
      purple = !purple;
    } else {
      altGreenPurp();
      purple = !purple;
    }
  }
  
  public void bouncingPurp(){
    new PrintCommand("running bouncing purple");
    for(int i = 0; i < ledBuffer.getLength(); i ++){
      for(int j = 0; j < ledBuffer.getLength(); j++){
        ledBuffer.setRGB(j, 0, 255, 0); //green
      }
      ledBuffer.setRGB(i, 185, 0, 255); //purple
      led.setData(ledBuffer);
    }
    /* i sure hope this works pls pls pls pls pls 
     * This is intended to return the purple pixel that's traveled to the last pixel back to the beginning
    */
    for(int i = ledBuffer.getLength() - 1; i > -1; i--){
        for(int j = 0; j < ledBuffer.getLength(); j++){
          ledBuffer.setRGB(j, 0, 255, 0); //green
        }
        ledBuffer.setRGB(i, 185, 0, 255); //purple
        led.setData(ledBuffer);
    }
    }
  

  public void bouncingGreen(){
    new PrintCommand("running bouncing green");
    for(int i = 0; i < ledBuffer.getLength(); i ++){
      for(int j = 0; j < ledBuffer.getLength(); j++){
        ledBuffer.setRGB(j, 185, 0, 255); //purple
      }
      ledBuffer.setRGB(i, 0, 255, 0); //green
      led.setData(ledBuffer);
    }
    for(int i = ledBuffer.getLength() - 1; i > -1; i--){
      for(int j = 0; j < ledBuffer.getLength(); j++){
        ledBuffer.setRGB(j, 185, 0, 255); //purple
      }
      ledBuffer.setRGB(i, 0, 255, 0); //green
      led.setData(ledBuffer);
    }
  }

  public void fillingPurple(Timer timer){
    new PrintCommand("running filling purple");
    while(time < ledBuffer.getLength()){
      for(int i = time; i > -1; i--){
        ledBuffer.setRGB(i, 185, 0, 255); //purple
      }
      led.setData(ledBuffer);
      if(timer.get()%0.5 > 0.25){
        time++;
        timer.restart();
      }
    }
  }
  
  public void extFillingPurple(Timer timer){
    new PrintCommand("running extended filling purple");
    while(time < ledBuffer.getLength()){
      for(int i = time; i > -1; i--){
        ledBuffer.setRGB(i, 185, 0, 255); //purple
      }
      led.setData(ledBuffer);
      if(timer.get()%0.5 > 0.25){
        time++;
        timer.restart();
      }
    }
    
  }

  public void groupedPurple(Timer timer){
    new PrintCommand("running grouped purple");
    while(time < ledBuffer.getLength()-6){
      for(int i = 0; i < 6; i++){
        ledBuffer.setRGB(time+i, 185, 0, 255); //purple
      }
      led.setData(ledBuffer);
      if(timer.get() > 0.25){
        time++;
        timer.restart();
      }
    }
  }

  public void altPurple(){
    new  PrintCommand("running alt purple");
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(time+i, 185, 0, 255); //purple
    }
  }

  public void altGreen(){
    new  PrintCommand("running alt purple");
    for(int i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 0, 255, 0); //green
    }
  }

  public void flashingPurple(Timer timer){
    new PrintCommand("running flashing purple");
    if(timer.get()%0.5 < 0.25){
      System.out.println("purple");
      altPurple();
      led.setData(ledBuffer);
    } else {
      System.out.println("not purple");
      showProgramCleanUp();
    }
    
  }

  public void flashingPurpleGreen (Timer timer){
    new PrintCommand("running flashing purple green");
    if(timer.get() < 0.25){
      if(purple){
        new PrintCommand("purple");
        altPurple();
        led.setData(ledBuffer);
      } else{
        new PrintCommand("green");
        altGreen();
        led.setData(ledBuffer);
      }
    } else {
      System.out.println("not purple");
      showProgramCleanUp();
    }
   
    if(timer.get()>0.5){
      timer.restart();
      purple = !purple;
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
