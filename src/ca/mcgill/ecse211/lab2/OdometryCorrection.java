package ca.mcgill.ecse211.lab2;

import static ca.mcgill.ecse211.lab2.Resources.*;
import lejos.hardware.Sound;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static int count = 0;                         // Number of black lines passed
  private static double X, Y, theta, thetaR;
  private static int NbLines = 3;                       // Number of black lines before turning
  private static float[] rgbArray = new float [1];
  private static int rgbThreshold = 35;

  /*
   * Here is where the odometer correction code should be run.
   */
  public void run() {
    long correctionStart, correctionEnd;
    
    while (true) {
      correctionStart = System.currentTimeMillis();
      theta = odometer.getXYT()[2];
      thetaR = theta * Math.PI / 180;        // Theta in radian
      colorSensor.getRedMode().fetchSample(rgbArray, 0);    // Gets the red value
      
      if ((int)(rgbArray[0]*100) < rgbThreshold) {                    // If statement runs when a black line is detected
          count++;
          Sound.beep();                                   // Indicator to check if a black line is detected
          
          if (theta >= 350 || theta <= 10) {                // Heading in positive Y-axis
            Y = TILE_SIZE * count - Math.abs(OFFSET * Math.cos(thetaR));                     // "OFFSET * cos(theta * pi/180)" 
            odometer.setY(Y);                                                                // represents the offset in the y direction since the robot might not always be facing completely foward
          }
          else if (Math.abs(theta - 90) <= 10) {            // Heading positive X-axis
            X = TILE_SIZE * count - Math.abs(OFFSET * Math.sin(thetaR));
            odometer.setX(X);
          }
          else if (Math.abs(theta - 180) <= 10) {           // Heading negative Y-axis
            Y = TILE_SIZE * (NbLines - (count - 1)) + Math.abs(OFFSET * Math.cos(thetaR));   // Add the offset since heading in negative direction
            odometer.setY(Y);
          }
          else if (Math.abs(theta - 270) <= 10) {           // Heading negative X-axis
            X = TILE_SIZE * (NbLines - (count - 1)) + Math.abs(OFFSET * Math.sin(thetaR));
            odometer.setX(X);
          }
          if (count >= 3) {                                 // count is reset since it turns after detecting 3 lines
            count = 0;
        }
      }

      // this ensures the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        Main.sleepFor(CORRECTION_PERIOD - (correctionEnd - correctionStart));
      }
    }
  }
  
}
