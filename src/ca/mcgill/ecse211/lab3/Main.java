package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
// static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.lab3.Resources.*;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;

public class Main {
  
  /**
   * The main entry point.
   * 
   * @param args
   */
  public static void main(String[] args) {
    
    /*
     * Start the 3 essential threads (Poller, Odometer and Display)
     */
    new Thread(new UltrasonicPoller()).start();
    new Thread(odometer).start();
    new Thread(new Display()).start();
    Button.waitForAnyPress();

    (new Thread() {
      public void run() {
          Navigation.travelTo(1, 3);        // Map 1
          Navigation.travelTo(2, 2);
          Navigation.travelTo(3, 3);
          Navigation.travelTo(3, 2);
          Navigation.travelTo(2, 1);

//          Navigation.travelTo(2, 2);      // Map 2
//          Navigation.travelTo(1, 3);
//          Navigation.travelTo(3, 3);
//          Navigation.travelTo(3, 2);
//          Navigation.travelTo(2, 1);

//          Navigation.travelTo(2, 1);      // Map 3
//          Navigation.travelTo(3, 2);
//          Navigation.travelTo(3, 3);
//          Navigation.travelTo(1, 3);
//          Navigation.travelTo(2, 2);
          
//          Navigation.travelTo(1, 2);      // Map 4
//          Navigation.travelTo(2, 3);
//          Navigation.travelTo(2, 1);
//          Navigation.travelTo(3, 2);
//          Navigation.travelTo(3, 3);
      }
    }).start();
    
    
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    } // do nothing
    System.exit(0);
    
  }


  /**
   * Sleeps current thread for the specified duration.
   * 
   * @param duration sleep duration in milliseconds
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }

}
