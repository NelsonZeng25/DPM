package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.utility.Delay;

public class LightLocalizer {

  // private static double[] angles = {0, 0, 0, 0}; // Array used to store the angles of the lines
  // private static int angleIndex = 0; // Int to see how many points it detected
  private static double initialRedValue; // Initial red value of board
  private static float[] rgbArray1 = new float[1]; // Variable used to get red value
  private static float[] rgbArray2 = new float[1]; // Variable used to get red value
  private static int rgbThreshold = 45; // Threshold to detect lines

  /**
   * Method used to locate the precise position of the (1,1) and orient the robot to 0 degree. 1.
   * Turn robot to 45 degrees since we start on 45 degree line. 2. Move forward until light sensor
   * hits line. Then move backwards by the light sensor offset (My sensor is placed on the back of
   * my robot). 3. Rotate and store angle for every black line it detects (expected number of
   * values: 4). 4. Use math formula to find displacement needed to reach (1,1) precisely. 5. Set
   * Odometer with true values and travel to (1,1) and turn to 0 degree.
   */
  public static void localize() {
    leftMotor.setSpeed(LOCALIZE_SPEED);
    rightMotor.setSpeed(LOCALIZE_SPEED);
    
    initialRedValue = getRedValue1();
    Navigation.turnTo(45);
    
    while (Math.abs(getRedValue1() - initialRedValue) < rgbThreshold) {
      leftMotor.forward();
      rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    leftMotor.rotate(-Navigation.convertDistance(LIGHTSENSOR_OFFSET), true);
    rightMotor.rotate(-Navigation.convertDistance(LIGHTSENSOR_OFFSET), false);
    Navigation.turnTo(180);

    leftMotor.setSpeed(100);
    rightMotor.setSpeed(100);
    sensorCorrect();
    leftMotor.rotate(Navigation.convertAngle(90.0), true);
    rightMotor.rotate(Navigation.convertAngle(-90.0), false);

    leftMotor.setSpeed(LOCALIZE_SPEED);
    rightMotor.setSpeed(LOCALIZE_SPEED);
    sensorCorrect();

  }

  private static void sensorCorrect() {
    
    leftMotor.backward();
    rightMotor.backward();

    while (true) {
      if (Math.abs(getRedValue1() - initialRedValue) > rgbThreshold) {
        leftMotor.stop(true);
        rightMotor.stop(false);
        Delay.msDelay((long) 0.5);
        leftMotor.setSpeed(LOCALIZE_SPEED);
        rightMotor.setSpeed(LOCALIZE_SPEED);
        while (true) {
          if (Math.abs(getRedValue2() - initialRedValue) > rgbThreshold) break;
          rightMotor.backward();
        }
        break;
        
      } else if (Math.abs(getRedValue2() - initialRedValue) > rgbThreshold) {
        rightMotor.stop(true);
        leftMotor.stop(false);
        Delay.msDelay((long) 0.5);
        leftMotor.setSpeed(LOCALIZE_SPEED);
        rightMotor.setSpeed(LOCALIZE_SPEED);
        while (true) {
          if (Math.abs(getRedValue1() - initialRedValue) > rgbThreshold) break;
          leftMotor.backward();
        }
        break;
      }
    }
    leftMotor.stop(true);
    rightMotor.stop(false);

    leftMotor.rotate(Navigation.convertDistance(TILE_SIZE / 2 - 12), true);
    rightMotor.rotate(Navigation.convertDistance(TILE_SIZE / 2 - 12), false);
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
  }

  /**
   * Gets the red value of the color sensor.
   * 
   * @return rgbArray[0] * 100
   */
  private static float getRedValue1() {
    colorSensor1.getRedMode().fetchSample(rgbArray1, 0);
    return rgbArray1[0] * 100;
  }

  private static float getRedValue2() {
    colorSensor2.getRedMode().fetchSample(rgbArray2, 0);
    return rgbArray2[0] * 100;
  }



}
