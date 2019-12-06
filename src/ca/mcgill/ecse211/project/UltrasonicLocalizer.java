package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * This class is used to localize the robot using the Ultrasonic Sensor
 */
public class UltrasonicLocalizer implements Runnable {

  /**
   * Stores the 2 angles from fallingEdge() or risingEdge()
   */
  private static double[] angles = {0, 0};
  
  /**
   * Correction delta theta used to correct the robot's angle
   */
  private static double deltaT;
  
  /**
   * Noise margin on when detecting falling edges or rising edges
   */
  private static final double NOISE_MARGIN = 10;
  
  /**
   * variable used to capture the direct output of the US sensor
   */
  private static float[] usData = new float[US_SENSOR.sampleSize()];
  
  /**
   * Variable to assign the filtered output of the US sensor
   */
  private static double distance;
  
  /**
   * Filter variable used to get the mean of the sample size
   */
  private static float filter;
  
  /**
   * How many samples we're polling before returning the distance
   */
  private static int sampleSize = 3;
  
  /**
   * Enables or disables the polling of the US sensor
   */
  private volatile boolean running = true;

  /**
   * Continuously get the distance of the USSensor until interrupt is detected
   */
  public void run() {
    running = true;
    while (running) {
      try {
        for (int i = 0; i < sampleSize; i++) {
          US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire data
          filter += usData[0] * 100;
          Thread.sleep(20);
        }
        distance = filter / sampleSize;
        filter = 0;
      } catch (InterruptedException e) {
        if (!running)
          break;
      }
    }
  }

  /**
   * Similar to the risingEdge method 
   * <p>1. It starts by checking if it's facing a WALL. If yes, it
   * will turn right until it's BIGGER than the band center. 
   * <p>2. After TURNING AWAY FROM a wall, it will continue to turn right until a falling edge is detected (i.e. a wall is detected). 
   * <p>3. The robot stops and records its angle. Repeat the 3 steps but in the opposite direction. This will
   * give you 2 angles where a Falling Edge is detected. 
   * <p>Use the math formula and add the deltaAngle to the current heading. This will update the current heading to its true heading.
   */
  public void fallingEdge() {

    // Checks for the case when robot is facing a wall (right)
    while (distance < BAND_CENTER + NOISE_MARGIN) {
      leftMotor.forward();
      rightMotor.backward();
    }
    // Continues to turn until wall is detected
    while (distance > BAND_CENTER - NOISE_MARGIN) {
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    angles[0] = odometer.getXYT()[2];

    while (distance < BAND_CENTER + NOISE_MARGIN) { // Turns opposite side (left)
      leftMotor.backward();
      rightMotor.forward();
    }
    while (distance > BAND_CENTER - NOISE_MARGIN) { // Continues to turn until wall is detected
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
   * @return the distance between the US sensor and an obstacle in cm
   */
  public static double getDistance() {
    return distance;
  }

  /**
   * Stops the provided thread by interrupting it
   * @param thread
   */
  public void stopThread(Thread thread) {
    running = false;
    thread.interrupt();
  }

}
