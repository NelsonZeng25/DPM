package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.BAND_CENTER;
import static ca.mcgill.ecse211.lab1.Resources.BAND_WIDTH;
import static ca.mcgill.ecse211.lab1.Resources.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab1.Resources.RIGHT_MOTOR;

public class PController extends UltrasonicController {

  private static final int MOTOR_SPEED = 200;
  private static final int speedThreshold = 75;             // the max difference of correction speed allowed
  private static final int p = 6;                           // P constant
  private static int distError;
  private static int deltaSpeed;
  
  public PController() {
    LEFT_MOTOR.setSpeed(MOTOR_SPEED);                       // Initialize motor rolling forward
    RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);

    distError = BAND_CENTER - this.distance;
    deltaSpeed = Math.abs(distError * p);

    if (Math.abs(distError) <= BAND_WIDTH) {                // If within error range, move foward
      LEFT_MOTOR.setSpeed(MOTOR_SPEED);                 
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
      RIGHT_MOTOR.forward();
      
    } else if (distError > 0) {                             // It's too close to the wall
      if (deltaSpeed > speedThreshold) {                    // Checks if the change of speed is higher than the threshold
        LEFT_MOTOR.setSpeed(MOTOR_SPEED + speedThreshold);  // Set Left motor to max speed
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED - 50);             // Set Right motor to specific constant (constant used for wider turns)
      } else {                                              // If less than threshold, increase/decrease speed of motors according to |error * p|
        LEFT_MOTOR.setSpeed(MOTOR_SPEED + deltaSpeed);
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED - deltaSpeed);
      }
      
      if (this.distance >= BAND_CENTER) {                   // If it's further than the band center, turn left slighty
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.forward();
      } else {                                              // If it's close to the wall, set Right motor backwards for a sharp turn
        LEFT_MOTOR.forward();
        RIGHT_MOTOR.backward();
      }
      
    } else if (distError < 0) {                             // It's too far from the wall
      if (deltaSpeed > speedThreshold) {                    // Checks again if the change of speed is higher than the threshold
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED + speedThreshold); // Set Right motor to max speed
        LEFT_MOTOR.setSpeed(MOTOR_SPEED - 50);              // Set Left motor to specific constant (constant used for wider turns)
      } else {                                              // If less than threshold, increase/decrease speed of motors according to |error * p|
        RIGHT_MOTOR.setSpeed(MOTOR_SPEED + deltaSpeed);
        LEFT_MOTOR.setSpeed(MOTOR_SPEED - deltaSpeed);
      }
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }

  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
