package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.utility.Delay;

/**
 * This class is used to localize the robot using the Light Sensors
 */
public class LightLocalizer implements Runnable {

  /**
   * Initial red value of the ground
   */
  public static double initialRedValue;

  /**
   * red value variables used to assigned the output of each light sensor
   */
  public static float redValue1, redValue2;

  /**
   * Enables or disables the polling of both light sensors
   */
  private volatile boolean running = true;

  /**
   * variables used to capture the direct output of the light sensor
   */
  private static float[] rgbArray1 = new float[1], rgbArray2 = new float[1];

  /**
   * Threshold to check if a line was detected
   */
  public static int rgbThreshold1 = 25;

  /**
   * Second threshold used to correct it's orientation when a line is detected (more sensitive than the first)
   */
  public static double rgbThreshold2 = 12.5;

  /**
   * Continuously get the red value of both color sensors to continously update the value of
   * redValue1 and redValue2 until an interrupt is detected
   */
  public void run() {
    running = true;
    while (running) {
      try {
        colorSensor1.getRedMode().fetchSample(rgbArray1, 0);
        colorSensor2.getRedMode().fetchSample(rgbArray2, 0);
        redValue1 = rgbArray1[0] * 100;
        redValue2 = rgbArray2[0] * 100;
        Thread.sleep(20);
      } catch (InterruptedException e) {
        if (!running)
          break;
      }
    }
  }

  /**
   * Method used to locate the precise position of the starting corner. <p>
   * 1. Turn robot to 45 degrees since we start on 45 degree line. <p>
   * 2. Move forward until light sensor hits line. Then move backwards by the light sensor offset divided 
   * by 3 This is because when the light sensor hits the line on the 45 degree line, the robot is offset/3 
   * away from the (1, 1) point <p>
   * 3. Right now, the robot should be at approx. the localization point. Turn the robot to 90 degree and run
   * the sensorCorrect() method to correct its x-orientation Turn the robot to 0 degree and run the
   * sensorCorrect() method to correct its y-orientation <p>
   * 4. We then move backwards by the light sensor offset since they're placed at the back of the robot 
   * This is to make sure the robot is always centered at (1,1) <p>
   * 5. Set Odometer with its corrected value (1,1) at 0 degree
   */
  public void localize() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);

    Delay.msDelay(200);
    initialRedValue = redValue1;
    Navigation.turnTo(45);

    while (Math.abs(redValue1 - initialRedValue) < rgbThreshold1) {
      leftMotor.forward();
      rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    Main.sleepFor(SLEEP_INTERVAL);
    leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET / 3), true);
    Main.sleepFor(SLEEP_INTERVAL);
    rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET / 3), false);
    Navigation.turnTo(90);

    sensorCorrect("forward");
    leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
    Main.sleepFor(SLEEP_INTERVAL);
    rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
    leftMotor.rotate(Navigation.convertAngle(-90), true);
    rightMotor.rotate(Navigation.convertAngle(90), false);

    sensorCorrect("forward");
    leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
    Main.sleepFor(SLEEP_INTERVAL);
    rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);

    Main.corner = (TEAM_NUMBER == redTeam) ? redCorner : greenCorner;
    switch (Main.corner) {
      case 0:
        odometer.setXYT(TILE_SIZE, TILE_SIZE, 0);
        break;
      case 1:
        odometer.setXYT(14 * TILE_SIZE, TILE_SIZE, 270);
        break;
      case 2:
        odometer.setXYT(14 * TILE_SIZE, 8 * TILE_SIZE, 180);
        break;
      case 3:
        odometer.setXYT(TILE_SIZE, 8 * TILE_SIZE, 90);
    }

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
  }

  /**
   * This method effectively corrects the robot's orientation by the using the 2 light sensors and
   * the lines. The end result is the robot perfectly perpendicular to the line <p>
   * 
   * When a light sensor detects a line, it will stop and depending on which sensor detects it
   * first, it will check if the other sensor has also detected a line. If yes, it means the robot
   * is straight. If not, stop the motor with the first sensor and rotate the motor with the other
   * sensor until the line is detected This will ensure that the robot is straight if the robot hit
   * the line at an angle. <p>
   * 
   * @param direction the direction of the correction (forward or backward)
   */
  public void sensorCorrect(String direction) {
    leftMotor.setAcceleration(LOCALIZE_ACCELERATION);
    rightMotor.setAcceleration(LOCALIZE_ACCELERATION);
    leftMotor.setSpeed(LOCALIZE_SPEED);
    rightMotor.setSpeed(LOCALIZE_SPEED);

    if (direction.contentEquals("forward")) {
      leftMotor.forward();
      rightMotor.forward();
    } else {
      leftMotor.backward();
      rightMotor.backward();
    }

    while (true) {
      if (Math.abs(redValue1 - initialRedValue) > rgbThreshold1) {
        leftMotor.stop(true);
        rightMotor.stop(false);
        Delay.msDelay(400);
        while (true) {
          if (Math.abs(redValue2 - initialRedValue) > rgbThreshold2) {
            leftMotor.stop(true);
            rightMotor.stop(false);
            break;
          }

          if (direction.contentEquals("forward")) {
            rightMotor.forward();
          } else {
            rightMotor.backward();
          }
        }
        break;

      } else if (Math.abs(redValue2 - initialRedValue) > rgbThreshold1) {
        leftMotor.stop(true);
        rightMotor.stop(false);
        Delay.msDelay(400);
        while (true) {
          if (Math.abs(redValue1 - initialRedValue) > rgbThreshold2) {
            leftMotor.stop(true);
            rightMotor.stop(false);
            break;
        }
          if (direction.contentEquals("forward")) {
            leftMotor.forward();
          } else {
            leftMotor.backward();
          }
        }
        break;
      }
    }
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
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
