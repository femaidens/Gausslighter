// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MorseLEDS extends SequentialCommandGroup {
  /** Creates a new MorseLEDS. */
  private final LED leds;
  public MorseLEDS(LED led) {
    this.leds = led;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //space between dots/dash within letter is a dit, all purple
    //space between letters is a dah, all purple
    //2265 = ..---  ..---  -....  .....
    addCommands(new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot, 2
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength),
    new GreenLEDS(led, LEDConstants.dahLength), //dah
    new PurpleLEDS(led, LEDConstants.ditLength),
    new GreenLEDS(led, LEDConstants.dahLength), //dah
    new PurpleLEDS(led, LEDConstants.ditLength),
    new GreenLEDS(led, LEDConstants.dahLength), //dah
    new PurpleLEDS(led, LEDConstants.dahLength),//---------------------
    new GreenLEDS(led, LEDConstants.ditLength), //dot, 2
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength),
    new GreenLEDS(led, LEDConstants.dahLength), //dah
    new PurpleLEDS(led, LEDConstants.ditLength),
    new GreenLEDS(led, LEDConstants.dahLength), //dah
    new PurpleLEDS(led, LEDConstants.ditLength),
    new GreenLEDS(led, LEDConstants.dahLength), //dah
    new PurpleLEDS(led, LEDConstants.dahLength), //-------------------
    new GreenLEDS(led, LEDConstants.dahLength), //dah, 6
    new PurpleLEDS(led, LEDConstants.ditLength),
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.dahLength), //--------------------
    new GreenLEDS(led, LEDConstants.ditLength), //dot, 5
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.ditLength), 
    new GreenLEDS(led, LEDConstants.ditLength), //dot
    new PurpleLEDS(led, LEDConstants.dahLength)
    );
  }
}
