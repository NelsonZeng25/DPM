package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Main.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import ca.mcgill.ecse211.project.Resources.Point;
import math.geom2d.Point2D;
import math.geom2d.conic.Circle2D;
import math.geom2d.line.Line2D;
import lejos.utility.Delay;

/**
 * This class is used to navigate the robot around the playing field
 */
public class Navigation implements Runnable {

  /**
   * Enables or disables the rotating motor and enables the obstacle avoidance detection in
   * travelTo()
   */
  public static boolean isAvoiding = false;

  /**
   * Stops the sensor motor without ending the thread if set to true
   */
  public static boolean stopped = false;

  /**
   * The starting angle of the sensor motor (should be 0)
   */
  public static int startAngle = (int) sensorMotor.getPosition();

  /**
   * How far the sensor rotates (in degrees) from the center (0)
   */
  public static int angle = 30;

  /**
   * How long the sensor motor waits before turning
   */
  private static int delay = 15;

  /**
   * Angle threshold used to determine if the robot has avoided the obstacle by seeing how the close
   * its angle is to the destination
   */
  private static int ANGLE_THRESHOLD = 20;

  /**
   * Ultrasonic Sensor distance
   */
  private static int USDistance;

  /**
   * Left-x delimiter of the island section
   */
  private static double A = (island.ll.x + 1) * TILE_SIZE;

  /**
   * Right-x delimiter of the island section
   */
  private static double B = (island.ur.x - 1) * TILE_SIZE;

  /**
   * Lower-y delimiter of the island section
   */
  private static double C = (island.ll.y + 1) * TILE_SIZE;

  /**
   * Upper-y delimiter of the island section
   */
  private static double D = (island.ur.y - 1) * TILE_SIZE;


  /**
   * Continously rotates the sensor motor back and forth and stops if stopped variable is true and
   * terminates if interrupted
   */
  public void run() {
    while (isAvoiding) {
      try {
        sensorMotor.rotateTo(startAngle + angle, false);
        Thread.sleep(delay);
        while (stopped);
        sensorMotor.rotateTo(startAngle, false);
        Thread.sleep(delay);
        while (stopped);
        sensorMotor.rotateTo(startAngle - angle, false);
        Thread.sleep(delay);
        while (stopped);
        sensorMotor.rotateTo(startAngle, false);
        Thread.sleep(delay);
        while (stopped);
      } catch (InterruptedException e) {
        if (!isAvoiding)
          break;
      }
    }
  }
  /**
   * Travels to coordinates x and y. Calls turnTo to set the angle first then sets the motors to
   * forward and rotates the distance required to get there.
   * <p>If isAvoiding is true (enabled), it will check if the Ultrasonic sensor distance is below a threshold. If it is, it means it has
   * detected an obstacle and will run the avoid() method.
   * <p>It will also if its location is close to the launch point and when it is, it will sweep the sensor again to check if an obstacle is on
   * that point. If there's an obstacle, it will get the next launch point and travel to that point
   * 
   * @param x the x-coordinate of waypoint
   * @param y the y-coordinate of waypoint
   */
  public static void travelTo(double x, double y) {
    double xi = odometer.getXYT()[0];
    double yi = odometer.getXYT()[1];
    double distance = distance(xi, yi, x * TILE_SIZE, y * TILE_SIZE);

    double headingTheta = computeTheta(x, y);

    turnTo(headingTheta);
    leftMotor.setAcceleration(ACCELERATION);
    rightMotor.setAcceleration(ACCELERATION);
    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    if (!isAvoiding) {
      rightMotor.rotate(convertDistance(distance), true);
      Main.sleepFor(SLEEP_INTERVAL);
      leftMotor.rotate(convertDistance(distance), false);
    } else {
      rightMotor.rotate(convertDistance(distance), true);
      Main.sleepFor(SLEEP_INTERVAL);
      leftMotor.rotate(convertDistance(distance), true);

      while (leftMotor.isMoving() && rightMotor.isMoving()) {
        USDistance = (int) UltrasonicLocalizer.getDistance();
        if (distance(odometer.getXYT()[0], odometer.getXYT()[1], launchPoint.x * TILE_SIZE, launchPoint.y * TILE_SIZE) <= TILE_SIZE){
          leftMotor.stop(true);
          rightMotor.stop(false);
          stopped = true;
          sensorMotor.rotateTo(startAngle - angle);
          double dist1 = UltrasonicLocalizer.getDistance();
          sensorMotor.rotateTo(startAngle);
          double dist2 = UltrasonicLocalizer.getDistance();
          sensorMotor.rotateTo(startAngle + angle);
          double dist3 = UltrasonicLocalizer.getDistance();
          if (dist1 < TILE_SIZE || dist2 < TILE_SIZE || dist3 < TILE_SIZE) {
            launchPoint = launchPointIterator.next();
            stopped = false;
            travelTo(launchPoint.x, launchPoint.y);
          } else {
            isAvoiding = false;
            travelTo(x, y);
          }
        } else if (USDistance < 25) {
            leftMotor.stop(true);
            rightMotor.stop(false);
            stopped = true;
            avoid(x, y);
        }
      }
    }
  }


  /**
   * Avoids the obstacle by going around it.
   * <p>It first determines at which sensor angle it detected the obstacle (startAngle + angle, startAngle, startAngle - angle) <br>
   * If the detected angle is not 0 (i.e the center), it means the obstacle is closer to the right/left so we turn the opposite
   * way If the detcted angle is 0, we sweep the sensor 1 extra time and check if there's a
   * Ultrasonic sensor distance difference between the 2 extremes of the sensor.
   * <p>If yes, it means the obstacle is closer on 1 end (right/left) so we turn the opposite way.
   * <p>If not, it means the obstacle is exactly perpendicular to the robot so we turn either way.
   * <p>This method will also check its section within the island and if it's close the edge, it will forcefully turn the
   * opposite way of the wall/water so this bypasses all the previous checks of the sensor. <br>
   * While it's going around the obstacle, it will constantly check it's facing relatively close to the
   * waypoint. When it does, it will stop and travel to that point since at this point we assume
   * that the obstacle has been avoided and while its travelling, it will call avoid(x, y) again if
   * another obstacle was detected.
   * 
   * @param x the x-coordinate of where it needs to go after avoiding
   * @param y the y-coordinate of where it needs to go after avoiding
   */
  private static void avoid(double x, double y) {
    int sensorAngle = (int) sensorMotor.getPosition();
    double turnAngle = 105 - USDistance;
    int speedDiff = 50;
    int distR = 0, distL = 0;
    String turn = edgeTurnDirection();


    if (sensorAngle == 0) {
      sensorMotor.rotateTo(startAngle - angle);
      distR = (int) UltrasonicLocalizer.getDistance();
      sensorMotor.rotateTo(startAngle + angle);
      distL = (int) UltrasonicLocalizer.getDistance();
    }

    if (turn.equals("")) {
      if (sensorAngle < 0 || distR < distL) {
        rightMotor.rotate(-convertAngle(turnAngle), true);
        Main.sleepFor(SLEEP_INTERVAL);
        leftMotor.rotate(convertAngle(turnAngle), false);

        leftMotor.setSpeed(200 + speedDiff);
        rightMotor.setSpeed(200 - speedDiff);

      } else {
        rightMotor.rotate(-convertAngle(turnAngle), true);
        Main.sleepFor(SLEEP_INTERVAL);
        leftMotor.rotate(convertAngle(turnAngle), false);
        
        leftMotor.setSpeed(200 - speedDiff);
        rightMotor.setSpeed(200 + speedDiff);
      }
    } else if (turn.equals("left")) {
      rightMotor.rotate(-convertAngle(turnAngle), true);
      Main.sleepFor(SLEEP_INTERVAL);
      leftMotor.rotate(convertAngle(turnAngle), false);

      leftMotor.setSpeed(200 + speedDiff);
      rightMotor.setSpeed(200 - speedDiff);

    } else {
      rightMotor.rotate(-convertAngle(turnAngle), true);
      Main.sleepFor(SLEEP_INTERVAL);
      leftMotor.rotate(convertAngle(turnAngle), false);

      leftMotor.setSpeed(200 - speedDiff);
      rightMotor.setSpeed(200 + speedDiff);
    }

    leftMotor.stop(true);
    rightMotor.stop(false);
    rightMotor.forward();
    Main.sleepFor(SLEEP_INTERVAL);
    leftMotor.forward();

    while (Math.abs(odometer.getXYT()[2] - computeTheta(x, y)) > ANGLE_THRESHOLD) Main.sleepFor(15);
    leftMotor.stop(true);
    rightMotor.stop(false);

    stopped = false;
    travelTo(x, y);
  }

  /**
   * This method makes the robot turn to an absolute theta using the minimal angle.
   * 
   * @param theta the theta of the direction we want to face
   */
  public static void turnTo(double theta) {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    leftMotor.stop(true);
    rightMotor.stop(false);

    double currentTheta = odometer.getXYT()[2];
    double deltaTheta = absoluteDeltaTheta(currentTheta, theta);
    Delay.msDelay(200);
    if (deltaTheta < 180) {
      rightMotor.rotate(convertAngle(deltaTheta), true);
      Main.sleepFor(SLEEP_INTERVAL);
      leftMotor.rotate(-convertAngle(deltaTheta), false);
    } else {
      rightMotor.rotate(-convertAngle(360 - deltaTheta), true);
      Main.sleepFor(SLEEP_INTERVAL);
      leftMotor.rotate(convertAngle(360 - deltaTheta), false);
    }
  }

  /**
   * turnTo method but with point input instead of angle
   * 
   * @param x x-coordinate of travel point
   * @param y y-coordinate of travel point
   */
  public static void turnTo(double x, double y) {
    turnTo(computeTheta(x, y));
  }

  /**
   * <p>Gives the difference in angle by checking from the current angle to the angle we want to
   * headTo. The method returns the minimal angle if the current angle is clock wise away from the
   * headingTo angle and below 180 degrees from headingTo If not, this method will give a "bad
   * angle" or "maximal angle" which is handled in the turnTo() method
   * 
   * <p>Simply put, if headingTo is left of current and <180 - Minimal angle (returns <180) if
   * headingTo is right of current and <180 - Maximal angle (returns >180)
   * 
   * @param current the current angle of the robot
   * @param headingTo the angle we want the robot to face
   * @return the delta angle between current angle and the angle it's heading to.
   */
  private static double absoluteDeltaTheta(double current, double headingTo) {
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
   * @param distance the distance we want to travel
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   * 
   * @param angle the angle we want to turn
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    double P_constant = 2.97;
    double input = P_constant * TRACK * angle / 360;
    return (int) ((180 * input) / (P_constant * WHEEL_RAD));
  }

  /**
   * This method computes the absolute theta to where it needs to go by using the odometer for its
   * initial position/theta and (x and y) for its displacement vector. <br>
   * Depending on where the vector is facing from the relative position of the robot, it will set the appropriate theta. The
   * appropriate theta is calculated with the 0 degree facing North and increasing degrees turning clock-wise.
   * 
   * @param x the target x-coordinate
   * @param y the target y-coordinate
   * @return the absolute theta of the destination
   */
  public static double computeTheta(double x, double y) {
    double xi = odometer.getXYT()[0];
    double yi = odometer.getXYT()[1];

    double dx = (x * TILE_SIZE) - xi;
    double dy = (y * TILE_SIZE) - yi;

    if (dx == 0 && dy > 0) { // Facing North
      return 0;
    } else if (dx == 0 && dy < 0) { // Facing South
      return 180;
    } else if (dx > 0 && dy == 0) { // Facing East
      return 90;
    } else if (dx < 0 && dy == 0) { // Facing West
      return 270;
    } else if (dx > 0 && dy > 0) { // North-East
      return 90 - Math.atan(dy / dx) * (180 / Math.PI);
    } else if (dx > 0 && dy < 0) { // South-East
      return 90 + Math.atan(-dy / dx) * (180 / Math.PI);
    } else if (dx < 0 && dy < 0) { // South-West
      return 180 + (90 - Math.atan(dy / dx) * (180 / Math.PI));
    } else { // North-West
      return 270 + Math.atan(-dy / dx) * (180 / Math.PI);
    }
  }

  /**
   * For the calculateTrajectory function, there are two main cases, either we are on the on the
   * lower or upper line of the rectangle, or on the left or right line of it. <br>
   * In each of these two cases, there are 3 separate cases, with 1, 2, or 3 coordinate points necessary to get to the
   * end point. 
   * <p> For the 1 coordinate point case, we tell the robot to go directly to the end point,
   * since it is on the same line as the starting point.
   * <p>For the 2 coordinate point case, we tell
   * the robot to first match either the X or Y coordinate to get on the same line as the end point,
   * and then go to the end point itself. 
   * <p>Finally, for the 3 coordinate point case, we first travel to one of the perpendicular lines, then travel 
   * along that line to get to the end line, and then we call travel to the end line itself. 
   * 
   * @param start start point
   * @param end end point
   * @return Sequence of waypoints to follow to reach the end point
   */
  public static List<Point> travelTrajectory(Point start, Point end) {
    List<Point> trajectory = new ArrayList<Point>();
    double lowerLineY = island.ll.y + 0.6;
    double upperLineY = island.ur.y - 0.6;
    double leftLineX = island.ll.x + 0.6;
    double rightLineX = island.ur.x - 0.6;
    double safezone1 = 0.8;
    double safezone2 = 0.3;
    
    if ((start.y < lowerLineY + safezone1) || (start.y > upperLineY - safezone1)) {
      // X is safe to move
      if (((end.y < lowerLineY + safezone1) && (start.y < lowerLineY + safezone1)) || ((end.y > upperLineY - safezone1) && (start.y > upperLineY - safezone1))) {
        // right on the line, go directly
        trajectory.add(end);
        return trajectory;
      }
     
      if ((end.x < leftLineX + safezone2) || (end.x > rightLineX - safezone2)) {
        // only need 2 moves to get there
        // first match the x, then match the y
        trajectory.add(new Point(end.x, start.y));
        trajectory.add(end);
        return trajectory;
      }
     
      // not 1 not 2, then we need 3 travels to get there
      // first go to the closest X line
      double closestX = (Math.abs(start.x - leftLineX) < Math.abs(start.x - rightLineX)) ? leftLineX : rightLineX;
      trajectory.add(new Point(closestX, start.y));
      // safe to match y now
      trajectory.add(new Point(closestX, end.y));
      // now just go
      trajectory.add(end);
      return trajectory;
    }
   
    // then we assume Y is safe to move
    if (((end.x < leftLineX + safezone1) && (start.x < leftLineX + safezone1)) || ((end.x > rightLineX - safezone1) && (start.x > rightLineX - safezone1))) {
      // right on the line, move directly
      trajectory.add(end);
      return trajectory;
    }
   
    if ((end.y < lowerLineY + safezone2) || (end.y > upperLineY - safezone2)) {
      // only need 2 moves
      trajectory.add(new Point(start.x, end.y));
      trajectory.add(end);
      return trajectory;
    }
   
    // not 1 not 2, then 3 travels
    // first go to the closest Y line
    double closestY = (Math.abs(start.y - upperLineY) < Math.abs(start.y - lowerLineY)) ? upperLineY : lowerLineY;
    trajectory.add(new Point(start.x, closestY));
    // safe to match y now
    trajectory.add(new Point(end.x, closestY));
    // now just go
    trajectory.add(end);
    return trajectory;
   
  }

  /**
   * The trajectoryDistance function is used to calculate a "cost" value for the robot to follow a
   * specific trajectory. 
   * <p> We input the start point, along with a trajectory from the
   * calculateTrajectory function. It will iterate through all the points in the trajectory list of
   * points, and compute the sum of all the euclidian distances from the start point and the
   * trajectory points. When we compare all the distances for all the possible launch point
   * trajectories, we can find the closest one to the robot and travel to that one.
   * 
   * @param start start point
   * @param trajectory sequence of points to reach an end point
   * @return "cost" or total distance to reach the end point using wall hugging travel
   */
  public static double trajectoryDistance(Point start, List<Point> trajectory) {
    double distance = 0;
    for (Point pt : trajectory) {
      distance += distance(start.x, start.y, pt.x, pt.y);
    }
    return distance;
  }


  /**
   * Calculates the launch to the corresponding bin point <br>
   * <p>If we're using obstacle avoidance, this method creates multiple lines smaller than the island's width and height on
   * the top right edges of the play area. 
   * <br>We then create a circle with its center on the bin point with a radius of the 
   * robot's launch distance. The circumference of the circle is all the points that the robot can shoot from and the lines
   * represent the permitted points that are not out of bounds. So, we simply get the intersection points from the circle and
   * the lines (We imported a special Math.geom jar file to get the points). Then, we sort the list of intersection points by
   * its closest distance to the robot's.
   * <p>If we're not using obstacle avoidance, it means we're using the wall hugging approach where we just travel on the edges
   * to avoid obstacles. 
   * <br>We simply create 4 lines that shapes the island area but slightly smaller since these lines represent
   * where our robot can shoot from and we're only travelling on the edges. We do the same thing with the circle with the
   * bin point and find the corresponding interesection points. The list of points is not sorted for this approach.
   * 
   * @return a list of viable launch points (sorted if using obstacle avoidance)
   */
  public static Collection<Point> findLaunchPoints() {
    bin.x = (TEAM_NUMBER == redTeam) ? redBin.x : greenBin.x;
    bin.y = (TEAM_NUMBER == redTeam) ? redBin.y : greenBin.y;

    Collection<Point> launchPoints = new ArrayList<Point>();
    Circle2D binCircle = new Circle2D(bin.x, bin.y, SHOOTING_RANGE);
    double rectx1, rectx2, recty1, recty2;

    // Gets the other launch points
    if (!isAvoiding) {
      rectx1 = island.ll.x + 0.6;
      rectx2 = island.ur.x - 0.6;
      recty1 = island.ll.y + 0.6;
      recty2 = island.ur.y - 0.6;
      Line2D[] lines = {new Line2D(rectx1, recty1, rectx2, recty1),
          new Line2D(rectx1, recty2, rectx2, recty2), new Line2D(rectx1, recty1, rectx1, recty2),
          new Line2D(rectx2, recty1, rectx2, recty2)};
      for (Line2D line : lines) {
        for (Point2D point : binCircle.intersections(line))
          launchPoints.add(new Point(point.getX(), point.getY()));
      }
    } else {
      // Gets shortest distance to bin launch point if it exists
      Line2D shortestLine = new Line2D(odometer.getXYT()[0] / TILE_SIZE,
          odometer.getXYT()[1] / TILE_SIZE, bin.x, bin.y);
      if (binCircle.intersections(shortestLine).size() != 0) {
        for (Point2D point : binCircle.intersections(shortestLine))
          launchPoints.add(new Point(point.getX(), point.getY()));
      }

      for (double i = 1; i < 1.5; i += 0.5) {
        rectx1 = island.ll.x + i;
        rectx2 = island.ur.x - i;
        recty1 = island.ll.y + i;
        recty2 = island.ur.y - i;
        Line2D[] lines = {
            new Line2D(rectx1, recty1, rectx2, recty1),
            new Line2D(rectx1, recty1, rectx1, recty2),
            new Line2D(rectx1, recty2, rectx2, recty2),
            new Line2D(rectx2, recty1, rectx2, recty2)};
        for (Line2D line : lines) {
          for (Point2D point : binCircle.intersections(line))
            launchPoints.add(new Point(point.getX(), point.getY()));
        }
        launchPoints = sortByClosest(launchPoints);
      }
    }
    return launchPoints;
  }

  /**
   * Sorts the launch points by its distance from the robot's location. This method sorts using a
   * hashmap with each entry as <Point, Double> where Double is the distance between the point and
   * the robot.
   * 
   * @param launchPoints
   * @return sorted launchPoints
   */
  @SuppressWarnings("unchecked")
  private static Collection<Point> sortByClosest(Collection<Point> launchPoints) {
    double x = odometer.getXYT()[0];
    double y = odometer.getXYT()[1];
    Collection<Point> result = new ArrayList<Point>();
    HashMap<Point, Double> list = new HashMap<Point, Double>();
    for (Point point : launchPoints) {
      list.put(point, distance(x, y, point.x * TILE_SIZE, point.y * TILE_SIZE));
    }
    Object[] sortedList = list.entrySet().toArray();
    Arrays.sort(sortedList, new Comparator<Object>() {
      public int compare(Object o1, Object o2) {
        return ((Map.Entry<Point, Double>) o1).getValue()
            .compareTo(((Map.Entry<Point, Double>) o2).getValue());
      }
    });
    for (Object entry : sortedList)
      result.add(((Map.Entry<Point, Double>) entry).getKey());
    return result;
  }

  /**
   * Checks which section the robot is within the Island The sections are numbered and in this
   * format: <p>3 4 5 <br>2 9 6 <br>1 8 7 <br>where 3 4 5 is the top row and 1 8 7 is the bottom row of the island
   * 
   * <p>This method is used to check if the robot is close the edges
   * 
   * @return section number
   */
  private static int findSection() {
    double x = odometer.getXYT()[0];
    double y = odometer.getXYT()[1];

    if (x < A) {
      if (y < C)
        return 1;
      else if (y > C && y < D)
        return 2;
      else
        return 3;
    } else if (x > A && x < B) {
      if (y > D)
        return 4;
      else if (y > C && y < D)
        return 9;
      else
        return 8;
    } else {
      if (y > D)
        return 5;
      else if (y > C && y < D)
        return 6;
      else
        return 7;
    }
  }

  /**
   * Returns which way the robot should turn if it's on the edges of the island
   * 
   * @return left or right as a String
   */
  private static String edgeTurnDirection() {
    int section = findSection();
    double angle = odometer.getXYT()[2];

    switch (section) {
      case 1:
        return (angle > 45 && angle < 225) ? "left" : "right";
      case 2:
        return (angle > 90 && angle < 270) ? "left" : "right";
      case 3:
        return (angle > 135 && angle < 315) ? "left" : "right";
      case 4:
        return (angle > 0 && angle < 180) ? "right" : "left";
      case 5:
        return (angle > 45 && angle < 225) ? "right" : "left";
      case 6:
        return (angle > 90 && angle < 270) ? "right" : "left";
      case 7:
        return (angle > 135 && angle < 315) ? "right" : "left";
      case 8:
        return (angle > 0 && angle < 180) ? "left" : "right";
      case 9:
        return "";
      default:
        return "";
    }
  }

  /**
   * The distance between 2 points
   * 
   * @param x1 x coordinate of Point 1
   * @param y1 y coordinate of Point 1
   * @param x2 x coordinate of Point 2
   * @param y2 y coordinate of Point 2
   * @return distance
   */
  public static double distance(double x1, double y1, double x2, double y2) {
    return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  /**
   * Stops the provided thread by interrupting it
   * 
   * @param thread
   */
  public void stopThread(Thread thread) {
    isAvoiding = false;
    thread.interrupt();
  }
}
