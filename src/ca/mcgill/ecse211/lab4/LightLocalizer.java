package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Sound;

public class LightLocalizer {

  private static double[] angles = {0, 0, 0, 0};        // Array used to store the angles of the black lines
  private static int angleIndex = 0;                    // Int to see how many points it detected
  private static double initialRedValue;                // Initial red value of board
  private static float[] rgbArray = new float[1];       // variable used to get red value
  private static int rgbThreshold = 20;                 // Threshold to detect lines

  /**
   * Method used to locate the precise position of the (1,1) and orient the robot to 0 degree
   * 1. Turn robot to 45 degrees since we start on 45 degree line
   * 2. Move forward until light sensor hits line. Then move backwards by the light sensor offset (My sensor is placed on the back of my robot)
   * 3. Rotate and store angle for every black line it detects (expected number of values: 4)
   * 4. Use math formula to find displacement needed to reach (1,1) precisely
   * 5. Set Odometer with true values and travel to (1,1) and turn to 0 degree
   */
  public static void localize() {
    // 1
    Navigation.turnTo(45);
    
    //2
    // Gets initial red value to compare it with the reading when it hits a black line
    // This method works with the blue tiles since it's relative
    initialRedValue = getRedValue();
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    // Will continue to turn until black line is detected
    while (Math.abs(getRedValue() - initialRedValue) < rgbThreshold) {
      leftMotor.forward();
      rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.rotate(-Navigation.convertDistance(LIGHTSENSOR_OFFSET), true);
    rightMotor.rotate(-Navigation.convertDistance(LIGHTSENSOR_OFFSET), false);
    
    // 3
    // Robot continues to turn until all 4 lines are detected
    leftMotor.backward();
    rightMotor.forward();
    while (angleIndex < 4) {
      // When it hits a black line, it records the angle
      if (Math.abs(getRedValue() - initialRedValue) > rgbThreshold) { 
        angles[angleIndex] = odometer.getXYT()[2];
        angleIndex++;
        Sound.beep();
      }
    }
    leftMotor.stop(true);
    rightMotor.stop(false);

    // 4
    // For the angles recorded, 0 = y-point, 1 = x-point, 2 = y-point, 3 = x-point 
    double deltaY = angles[2] - angles[0];
    double deltaX = angles[3] - angles[1];
    double x = LIGHTSENSOR_OFFSET * Math.cos(deltaX * Math.PI / (180 * 2));
    double y = LIGHTSENSOR_OFFSET * Math.cos(deltaY * Math.PI / (180 * 2));
    
    // 5
    // Set x and y values to odometer accordingly
    odometer.setX(TILE_SIZE - x);
    odometer.setY(TILE_SIZE - y);

    Navigation.travelTo(1, 1);
    Navigation.turnTo(0);
  }

  /**
   * Gets the red value of the color sensor
   * @return rgbArray[0] * 100
   */
  private static float getRedValue() {
    colorSensor.getRedMode().fetchSample(rgbArray, 0);
    return rgbArray[0] * 100;
  }



}
