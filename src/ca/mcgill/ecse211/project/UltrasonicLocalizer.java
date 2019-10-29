package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

public class UltrasonicLocalizer {

  private static double[] angles = {0, 0}; // Stores the 2 angles detected for each method
  private static float[] usData = new float[US_SENSOR.sampleSize()]; // Used to store USDistance
  private static double distance, deltaT; // Distance variable for USSensor and deltaT variable
  private static final double NOISE_MARGIN = 5;

  /**
   * Similar to the risingEdge method 
   * 1. It starts by checking if it's facing a WALL. If yes, it will turn right until it's BIGGER than the band center. 
   * 2. After TURNING AWAY FROM a wall, it will continue to turn right until a falling edge is detected (i.e. a wall is detected). 
   * 3. The robot stops and records its angle. 
   * Repeat the 3 steps but in the opposite direction. 
   * This will give you 2 angles where a Falling Edge is detected. 
   * Use the math formula and add the deltaAngle to the current heading. 
   * This will update the current heading to its true heading. 
   */
  public static void fallingEdge() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    // Checks for the case when robot is facing a wall (right)
    while (getDistance() < BAND_CENTER + NOISE_MARGIN) {
      leftMotor.forward();
      rightMotor.backward();
    }
    // Continues to turn until wall is detected
    while (getDistance() > BAND_CENTER - NOISE_MARGIN) {
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    angles[0] = odometer.getXYT()[2];

    while (getDistance() < BAND_CENTER + NOISE_MARGIN) { // Turns opposite side (left)
      leftMotor.backward();
      rightMotor.forward();
    }
    while (getDistance() > BAND_CENTER - NOISE_MARGIN) { // Continues to turn until wall is detected
      leftMotor.backward();
      rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    angles[1] = odometer.getXYT()[2];

    if (angles[0] > angles[1]) {
      deltaT = 45 - (angles[0] + angles[1]) / 2.0;
    } else {
      deltaT = 225 - (angles[0] + angles[1]) / 2.0;
    }

    odometer.update(0, 0, deltaT);

    leftMotor.stop(true);
    rightMotor.stop(false);
  }

  /**
   * Returns the distance between the US sensor and an obstacle in cm.
   *
   * @return the distance between the US sensor and an obstacle in cm
   */
  public static double getDistance() {
    US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire data
    distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
    distance = distance > 150 ? 150 : distance; // Small filter to filter out very large numbers
    return distance;
  }

}
