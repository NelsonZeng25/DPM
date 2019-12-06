package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.utility.Delay;
// Static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.project.Resources.*;
import static ca.mcgill.ecse211.project.Navigation.*;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import ca.mcgill.ecse211.project.Resources.Point;

/**
 * The main driver class for the lab.
 */
public class Main {

  private static UltrasonicLocalizer UL = new UltrasonicLocalizer();
  private static LightLocalizer LL = new LightLocalizer();
  private static Navigation NAV = new Navigation();

  private static Thread USSensor = new Thread(UL);
  private static Thread LightSensor = new Thread(LL);
  private static Thread NavSensor = new Thread(NAV);
  private static Thread Odometer = new Thread(odometer);

  /**
   * Bin location (sets the proper bin if it's red or green)
   */
  public static Point bin = new Point(0, 0);
  
  /**
   * Starting corner location
   */
  public static Point origin = new Point(0, 0);

  /**
   * Point to localize in starting zone before heading in tunnel
   */
  public static Point tunnelBefore = new Point(0, 0);

  /**
   * Point to localize in island after going through tunnel
   */
  public static Point tunnelAfter = new Point(0, 0);

  /**
   * The corner we start on
   */
  public static int corner = 0;
  
  /**
   * The starting zone lower left corner point
   */
  public static Point zone_ll = new Point(0,0);
  
  /**
   * The starting zone upper right corner point
   */
  public static Point zone_ur = new Point(0,0);
  
  /**
   * Starting location
   */
  public static Point home_point = new Point(0, 0);
  
  /**
   * Launch point of the robot
   */
  public static Point launchPoint = new Point(-10, -10);

  /**
   * Array of viable launch points
   */
  public static ArrayList<Point> launchPoints;

  /**
   * Iterator for the array of launch points
   */
  public static Iterator<Point> launchPointIterator;

  /**
   * The main entry point.
   *
   * @param args
   */
  public static void main(String[] args) {

    Odometer.start();

    (new Thread() {
      public void run() {
        // Set up motor settings and bin location
        leftMotor.setAcceleration(ACCELERATION);
        rightMotor.setAcceleration(ACCELERATION);
        leftMotor.setSpeed(FORWARD_SPEED);
        rightMotor.setSpeed(FORWARD_SPEED);
        sensorMotor.setAcceleration(1000);
        sensorMotor.setSpeed(175);

        Point ll, ur;
        if (TEAM_NUMBER == redTeam) {
          corner = redCorner;
          zone_ll = red.ll;
          zone_ur = red.ur;
          ll = tnr.ll;
          ur = tnr.ur;
          bin = redBin;
        } else {
          corner = greenCorner;
          zone_ll = green.ll;
          zone_ur = green.ur;
          ll = tng.ll;
          ur = tng.ur;
          bin = greenBin;
        }
        boolean isTall = Math.abs(ll.x - ur.x) < Math.abs(ll.y - ur.y);

        // Get the before and after tunnel coordinates
        if (isTall) {
          switch (corner) {
            case 0:
              if (ll.x == zone_ll.x) {
                tunnelBefore = new Point(ll.x + 1, ll.y - 1);
                tunnelAfter = new Point(ur.x, ur.y + 1);
              } else {
                tunnelBefore = new Point(ll.x, ll.y - 1);
                tunnelAfter = new Point(ur.x - 1, ur.y + 1);
              }
              break;
            case 1:
              if (ur.x == zone_ur.x) {
                tunnelBefore = new Point(ll.x, ll.y - 1);
                tunnelAfter = new Point(ur.x - 1, ur.y + 1);
              } else {
                tunnelBefore = new Point(ll.x + 1, ll.y - 1);
                tunnelAfter = new Point(ur.x, ur.y + 1);
              }
              break;
            case 2:
              if (ur.x == zone_ur.x) {
                tunnelBefore = new Point(ur.x - 1, ur.y + 1);
                tunnelAfter = new Point(ll.x, ll.y - 1);
              } else {
                tunnelBefore = new Point(ur.x, ur.y + 1);
                tunnelAfter = new Point(ll.x + 1, ll.y - 1);
              }
              break;
            case 3:
              if (ll.x == zone_ll.x) {
                tunnelBefore = new Point(ur.x, ur.y + 1);
                tunnelAfter = new Point(ll.x + 1, ll.y - 1);
              } else {
                tunnelBefore = new Point(ur.x - 1, ur.y + 1);
                tunnelAfter = new Point(ll.x, ll.y - 1);
              }
              break;
          }
        } else {
          switch (corner) {
            case 0:
              if (ll.y == zone_ll.y) {
                tunnelBefore = new Point(ll.x - 1, ll.y + 1);
                tunnelAfter = new Point(ur.x + 1, ur.y);
              } else {
                tunnelBefore = new Point(ll.x - 1, ll.y);
                tunnelAfter = new Point(ur.x + 1, ur.y - 1);
              }
              break;
            case 1:
              if (ll.y == zone_ll.y) {
                tunnelBefore = new Point(ur.x + 1, ur.y);
                tunnelAfter = new Point(ll.x - 1, ll.y + 1);
              } else {
                tunnelBefore = new Point(ur.x + 1, ur.y - 1);
                tunnelAfter = new Point(ll.x - 1, ll.y);
              }
              break;
            case 2:
              if (ur.y == zone_ur.y) {
                tunnelBefore = new Point(ur.x - 1, ur.y + 1);
                tunnelAfter = new Point(ll.x, ll.y - 1);
              } else {
                tunnelBefore = new Point(ur.x + 1, ur.y);
                tunnelAfter = new Point(ll.x - 1, ll.y + 1);
              }
              break;
            case 3:
              if (ur.y == zone_ur.y) {
                tunnelBefore = new Point(ll.x - 1, ll.y);
                tunnelAfter = new Point(ur.x + 1, ur.y - 1);
              } else {
                tunnelBefore = new Point(ll.x - 1, ll.y + 1);
                tunnelAfter = new Point(ur.x + 1, ur.y);
              }
              break;
          }
        }

        // Ultrasonic sensor localization
        USSensor.start();
        UL.fallingEdge();
        UL.stopThread(USSensor);

        // Light sensor localization
        LightSensor.start();
        LL.localize();
        Sound.beep();
        Sound.beep();
        Sound.beep();

        // Travel to point next to tunnel and localize
        travelTo(tunnelBefore.x, tunnelBefore.y);
        turnTo(270);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        Main.sleepFor(100);
        turnTo(0);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        odometer.setXYT(tunnelBefore.x * TILE_SIZE, tunnelBefore.y * TILE_SIZE, 0);

        // Travel through tunnel
        if (isTall) {
          travelTo((ll.x + ur.x) / 2, tunnelBefore.y);
          turnTo((ll.x + ur.x) / 2, tunnelAfter.y);
          LL.sensorCorrect("forward");
          if (corner == 0 || corner == 1) {
            odometer.setTheta(0);
          } else {
            odometer.setTheta(180);
          }
          LL.stopThread(LightSensor);
          travelTo((ll.x + ur.x) / 2, tunnelAfter.y);
        } else {
          travelTo(tunnelBefore.x, (ll.y + ur.y) / 2);
          turnTo(tunnelAfter.x, (ll.y + ur.y) / 2);
          LL.sensorCorrect("forward");
          if (corner == 0 || corner == 3) {
            odometer.setTheta(90);
          } else {
            odometer.setTheta(270);
          }
          LL.stopThread(LightSensor);
          travelTo(tunnelAfter.x, (ll.y + ur.y) / 2);
        }

        // Localize to point after the tunnel
        LightSensor = new Thread(LL);
        LightSensor.start();
        Delay.msDelay(500);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        if (isTall) {
          if (corner == 0 || corner == 1)
            odometer.setXYT(((ll.x + ur.x) / 2) * TILE_SIZE, tunnelAfter.y * TILE_SIZE, 0);
          else
            odometer.setXYT(((ll.x + ur.x) / 2) * TILE_SIZE, tunnelAfter.y * TILE_SIZE, 180);
        } else {
          if (corner == 0 || corner == 3)
            odometer.setXYT(tunnelAfter.x * TILE_SIZE, ((ll.y + ur.y) / 2) * TILE_SIZE, 90);
          else
            odometer.setXYT(tunnelAfter.x * TILE_SIZE, ((ll.y + ur.y) / 2) * TILE_SIZE, 270);
        }
        travelTo(tunnelAfter.x, tunnelAfter.y);

        turnTo(270);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        Main.sleepFor(SLEEP_INTERVAL);
        turnTo(0);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        odometer.setXYT(tunnelAfter.x * TILE_SIZE, tunnelAfter.y * TILE_SIZE, 0);
        LL.stopThread(LightSensor);
        

        // Start obstacle avoidance and get launch points
        isAvoiding = true; ///////////////////////////////////////////////////////////
        
        launchPoints = (ArrayList<Point>) findLaunchPoints();
        if (isAvoiding) {
          launchPointIterator = launchPoints.iterator();
          launchPoint = launchPointIterator.next();

          USSensor = new Thread(UL);
          USSensor.start();
          NavSensor.start();
          travelTo(Math.floor(launchPoint.x), Math.floor(launchPoint.y));
          LightSensor = new Thread(LL);
          LightSensor.start();
          turnTo(270);
          LL.sensorCorrect("forward");
          leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
          Main.sleepFor(SLEEP_INTERVAL);
          rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
          Main.sleepFor(SLEEP_INTERVAL);
          turnTo(0);
          LL.sensorCorrect("forward");
          leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
          Main.sleepFor(SLEEP_INTERVAL);
          rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
          LL.stopThread(LightSensor);
          odometer.setXYT(Math.floor(launchPoint.x) * TILE_SIZE,
              Math.floor(launchPoint.y) * TILE_SIZE, 0);

          while (true) {
            travelTo(launchPoint.x, launchPoint.y);
            turnTo(bin.x, bin.y);
            stopped = true;
            sensorMotor.rotateTo(startAngle - angle);
            double dist1 = UltrasonicLocalizer.getDistance();
            sensorMotor.rotateTo(startAngle);
            double dist2 = UltrasonicLocalizer.getDistance();
            sensorMotor.rotateTo(startAngle + angle);
            double dist3 = UltrasonicLocalizer.getDistance();
            if (dist1 < TILE_SIZE / 2 || dist2 < (TILE_SIZE / 2) / Math.cos(Math.toRadians(30))
                || dist3 < (TILE_SIZE / 2) / Math.cos(Math.toRadians(30))) {
              launchPoint = launchPointIterator.next();
            } else {
              break;
            }
            stopped = false;
          }
          Sound.beep();
          Sound.beep();
          Sound.beep();
          leftMotor.rotate(convertAngle(-10));
          launch();
          
          // Return back!
          travelTo(tunnelAfter.x, tunnelAfter.y);
          
          NAV.stopThread(NavSensor);
          UL.stopThread(USSensor);

        } else {
          leftMotor.rotate(convertDistance(-0.4 * TILE_SIZE), true);
          Main.sleepFor(SLEEP_INTERVAL);
          rightMotor.rotate(convertDistance(-0.4 * TILE_SIZE), false);
          Point returnPoint = new Point(odometer.getXYT()[0] / TILE_SIZE, odometer.getXYT()[1] / TILE_SIZE);
          
          Point start =
              new Point(odometer.getXYT()[0] / TILE_SIZE, odometer.getXYT()[1] / TILE_SIZE);
          List<Point> shortestTraj = null;
          double cheapestCost = 9999;
          for (Point pt : launchPoints) {
            List<Point> traj = travelTrajectory(start, pt);
            double cost = trajectoryDistance(start, traj);
            if (cost < cheapestCost) {
              cheapestCost = cost;
              shortestTraj = traj;
            }
          }
          launchPoint = shortestTraj.get(shortestTraj.size() - 1);
          for (Point pt : shortestTraj) {
            travelTo(pt.x, pt.y);
          }
          turnTo(bin.x, bin.y);

          Sound.beep();
          Sound.beep();
          Sound.beep();
          leftMotor.rotate(convertAngle(-10));
          launch();

          // Return back!
          start = new Point(odometer.getXYT()[0] / TILE_SIZE, odometer.getXYT()[1] / TILE_SIZE);
          for (Point pt: travelTrajectory(start, returnPoint)) {
            travelTo(pt.x, pt.y);
          }
          travelTo(tunnelAfter.x, tunnelAfter.y);
        }
        
        LightSensor = new Thread(LL);
        LightSensor.start();
        Delay.msDelay(500);
        turnTo(270);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        Main.sleepFor(SLEEP_INTERVAL);
        turnTo(0);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        odometer.setXYT(tunnelAfter.x * TILE_SIZE, tunnelAfter.y * TILE_SIZE, 0);
        
        // Go through tunnel again
        if (isTall) {
          travelTo((ll.x + ur.x) / 2, tunnelAfter.y);
          turnTo((ll.x + ur.x) / 2, tunnelBefore.y);
          LL.sensorCorrect("forward");
          if (corner == 0 || corner == 1) {
            odometer.setTheta(180);
          } else {
            odometer.setTheta(0);
          }
          LL.stopThread(LightSensor);
          travelTo((ll.x + ur.x) / 2, tunnelBefore.y);
        } else {
          travelTo(tunnelAfter.x, (ll.y + ur.y) / 2);
          turnTo(tunnelBefore.x, (ll.y + ur.y) / 2);
          LL.sensorCorrect("forward");
          if (corner == 0 || corner == 3) {
            odometer.setTheta(270);
          } else {
            odometer.setTheta(90);
          }
          LL.stopThread(LightSensor);
          travelTo(tunnelBefore.x, (ll.y + ur.y) / 2);
        }
        
        // Go back to origin point
        LightSensor = new Thread(LL);
        LightSensor.start();
        Delay.msDelay(500);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        if (isTall) {
          if (corner == 0 || corner == 1)
            odometer.setXYT(((ll.x + ur.x) / 2) * TILE_SIZE, tunnelBefore.y * TILE_SIZE, 180);
          else
            odometer.setXYT(((ll.x + ur.x) / 2) * TILE_SIZE, tunnelBefore.y * TILE_SIZE, 0);
        } else {
          if (corner == 0 || corner == 3)
            odometer.setXYT(tunnelBefore.x * TILE_SIZE, ((ll.y + ur.y) / 2) * TILE_SIZE, 270);
          else
            odometer.setXYT(tunnelBefore.x * TILE_SIZE, ((ll.y + ur.y) / 2) * TILE_SIZE, 90);
        }
        travelTo(tunnelBefore.x, tunnelBefore.y);
        turnTo(270);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        Main.sleepFor(SLEEP_INTERVAL);
        turnTo(0);
        LL.sensorCorrect("forward");
        leftMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), true);
        Main.sleepFor(SLEEP_INTERVAL);
        rightMotor.rotate(Navigation.convertDistance(-LIGHTSENSOR_OFFSET), false);
        
        switch(corner) {
          case 0:
            home_point = new Point(0, 0);
            break;
          case 1:
            home_point = new Point(15, 0);
            break;
          case 2:
            home_point = new Point(15, 9);
            break;
          case 3:
            home_point = new Point(0, 9);
        }
        travelTo(home_point.x, home_point.y);
        leftMotor.stop(true);
        rightMotor.stop(false);
        Sound.beep();
        Sound.beep();
        Sound.beep();
        Sound.beep();
        Sound.beep();
      }
    }).start();

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }


  /**
   * Sets the appropriate Acceleration and Speed for the launching mechanism with the spring After
   * running this method, it will continue to rotate backwards until a button is pressed
   */
  public static void launch() {
    launcher.setAcceleration(6000);
    launcher.setSpeed(150);
    launcher.rotate(-720);
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
