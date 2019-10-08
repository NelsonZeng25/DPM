package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;

public class Navigation {
  
  /**
   * This method moves the robot to a certain location given by x and y which are tile points with origin in
   * the bottom left corner. It turns the robot to its destination and goes foward. It will stop whenever the 
   * US distance detected is lower than a threshold.
   * 
   * @param x
   * @param y
   */
  public static void travelTo(double x, double y) {
    double dx, dy, distance, headingTheta;
    double xi = odometer.getXYT()[0];
    double yi = odometer.getXYT()[1];
    dx = (x * TILE_SIZE) - xi;
    dy = (y * TILE_SIZE) - yi;
    distance = Math.sqrt(dx * dx + dy * dy);
    
    headingTheta = computeTheta(x, y);

    turnTo(headingTheta);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), true);
    
  }
  
  /**
   * This method makes the robot turn to a absolute theta with always a minimal angle
   * 
   * @param theta
   */
  public static void turnTo(double theta) {
    leftMotor.stop();
    rightMotor.stop();
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    double currentTheta = odometer.getXYT()[2];
    double deltaTheta = absoluteDeltaTheta(currentTheta, theta);
    
    if (deltaTheta < 180) {
      leftMotor.rotate(-convertAngle(deltaTheta), true);
      rightMotor.rotate(convertAngle(deltaTheta), false);
    } else {
      leftMotor.rotate(convertAngle(360 - deltaTheta), true);
      rightMotor.rotate(-convertAngle(360 - deltaTheta), false);
    }
  }

  /**
   * Gives the difference in angle by checking from the current angle to the angle we want to headTo.
   * The method returns the minimal angle if the current angle is clock wise away from the headingTo angle and below 180 degrees from headingTo
   * If not, this method will give a "bad angle" or "maximal angle" which is handled in the turnTo() method
   * 
   * Simply put, 
   * if headingTo is left of current and <180 - Minimal angle (returns <180)
   * if headingTo is right of current and <180 - Maximal angle (returns >180)
   * 
   * @param current
   * @param headingTo
   * @return the delta angle between current angle and the angle it's heading to. 
   */
  public static double absoluteDeltaTheta(double current, double headingTo) {
    if (current - headingTo < 0) {
      return (360 - headingTo) + current;
    } else if (current - headingTo > 0) {
      return current - headingTo;
    } else {
      return current;
    }
  }
  
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }
  
  
  /**
   * This method computes the absolute theta to where it needs to go by using the odometer for its initial position/theta
   * and (x and y) for its displacement vector. Depending on where the vector is facing from the relative position of the 
   * robot, it will set the appropriate theta.
   * The appropriate theta is caculated with the 0 degree facing North and increasing degrees turning clock-wise
   * @param x
   * @param y
   * @return the absolute theta of the destination
   */
  public static double computeTheta(double x, double y) {
    double dx, dy;
    double[] position = odometer.getXYT();
    double xi = position[0];
    double yi = position[1];
    
    dx = (x * TILE_SIZE) - xi;
    dy = (y * TILE_SIZE) - yi;
    
    if (dx == 0 && dy > 0) {            // Facing North
      return 0;
    }
    else if (dx == 0 && dy < 0) {       // Facing South
      return 180;
    }
    else if (dx > 0 && dy == 0) {       // Facing East
      return 90;
    }
    else if (dx < 0 && dy == 0) {       // Facing West
      return 270;
    }
    else if (dx > 0 && dy > 0) {                                    // North-East
      return 90 - Math.atan(dy / dx) * (180 / Math.PI);
    }
    else if (dx > 0 && dy < 0) {                                    // South-East
      return 90 + Math.atan(-dy / dx) * (180 / Math.PI);
    }
    else if (dx < 0 && dy < 0) {                                    // South-West
      return 180 + (90 - Math.atan(dy / dx) * (180 / Math.PI));
    }
    else {                                                          // North-West
      return 270 + Math.atan(-dy / dx) * (180 / Math.PI);
    }
  }
}