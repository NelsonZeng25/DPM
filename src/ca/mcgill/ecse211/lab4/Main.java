package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Button;

public class Main {
  private static boolean isRising;          // Boolean to check if RisingEdge is selected
  
  
  public static void main(String[] args) {  
    new Thread(odometer).start();                       // Start Odometer thread
    new Thread(new UltrasonicLocalizer()).start();      // Start USSensor thread
    
    int buttonChoice = chooseRisingOrFalling();         
    new Thread(new Display()).start();                  // Start Display thread
    if (buttonChoice == Button.ID_LEFT) {               // Set isRising to true if left button is clicked
      isRising = true;
    } else {
      isRising = false;
    }
    (new Thread() {                                     // Run seperate thread for risingEdge or fallingEdge
      public void run() {
        if (isRising) {
          UltrasonicLocalizer.risingEdge();
        } else {
          UltrasonicLocalizer.fallingEdge();
        }
      }
    }).start();
    
    Button.waitForAnyPress();                           // Wait for press to enable LightLocalizer
    LightLocalizer.localize();
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    } // do nothing
    System.exit(0);
  }
  
  /**
   * Method to display choice for user (Rising or Falling Edge)"
   * @return buttonChoice
   */
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
