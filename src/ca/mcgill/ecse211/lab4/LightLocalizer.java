package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;
import lejos.hardware.Sound;

public class LightLocalizer {
  // constants
  public static int DISTANCE_FROM_EDGE = 18;
  public static int ACCELERATION = 600;

  // defining the class variables
  private static double[] angles = {0, 0, 0, 0};
  private static int angleIndex = 0;
  private static double initialRedValue;
  private static float[] rgbArray = new float[1];
  private static int rgbThreshold = 20;


  // Using the light sensor to localize the robot



  public static void localize() {

    Navigation.turnTo(45);
    
    initialRedValue = getRedValue();
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    while (Math.abs(getRedValue() - initialRedValue) < rgbThreshold) {
      leftMotor.forward();
      rightMotor.forward();
    }

    // Once we have reached the black line, we stop.
    leftMotor.stop(true);
    rightMotor.stop(false);

    // Next, we will go back by a certain amount
    leftMotor.rotate(-convertDistance(LIGHTSENSOR_OFFSET), true);
    rightMotor.rotate(-convertDistance(LIGHTSENSOR_OFFSET), false);

    leftMotor.backward();
    rightMotor.forward();
    // This method records the angles at subsequent (4) hits in an array.
    while (angleIndex < 4) {
      // When it hits a line it beeps and records the angle
      if (Math.abs(getRedValue() - initialRedValue) > rgbThreshold) { 
        angles[angleIndex] = odometer.getXYT()[2];
        angleIndex++;
        Sound.beep();
      }
    }

    // Now that all the values have been recorded, we stop.
    leftMotor.stop(true);
    rightMotor.stop(false);

    // In this array the first element is: y line, second = first x point, third = second y, fourth
    // = second x
    // We then calculate the deltas at each of the to directions X and Y.
    double deltaY = angles[2] - angles[0];
    double deltaX = angles[3] - angles[1];

    // do trig to compute (0,0) and 0 degrees
    double x = LIGHTSENSOR_OFFSET * Math.cos(deltaX * Math.PI / (180 * 2));
    double y = LIGHTSENSOR_OFFSET * Math.cos(deltaY * Math.PI / (180 * 2));

    // set the position of the robot to where we are and an angle of 0.
    odometer.setX(TILE_SIZE - x);
    odometer.setY(TILE_SIZE - y);

    Navigation.travelTo(1, 1);
    Navigation.turnTo(0);
  }

  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }
  
  private static float getRedValue() {
    colorSensor.getRedMode().fetchSample(rgbArray, 0);
    // we define the brightness as the average of the magnitudes of R,G,B (really "Whiteness")
    return rgbArray[0] * 100;
  }



}
