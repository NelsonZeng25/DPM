package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Button;

public class Main {
  private static boolean isRising;
  public static void main(String[] args) {
    int buttonChoice;
    new Thread(odometer).start();
    new Thread(new UltrasonicLocalizer()).start();
    buttonChoice = chooseRisingOrFalling();
    new Thread(new Display()).start();
    if (buttonChoice == Button.ID_LEFT) {
      isRising = true;
    } else {
      isRising = false;
    }
    (new Thread() {
      public void run() {
        if (isRising) {
          UltrasonicLocalizer.risingEdge();
        } else {
          UltrasonicLocalizer.fallingEdge();
        }
      }
    }).start();
    
    Button.waitForAnyPress();
    LightLocalizer.localize();
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    } // do nothing
    System.exit(0);
  }
  
  private static int chooseRisingOrFalling() {
    int buttonChoice;
    Display.showText("< Left | Right >",
                     "       |        ",
                     "Rising | Falling",
                     "edge   | edge   ");
    
    do {
      buttonChoice = Button.waitForAnyPress(); // left or right press
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
    return buttonChoice;
  }
}
