package ca.mcgill.ecse211.lab1;

import static ca.mcgill.ecse211.lab1.Resources.BAND_CENTER;
import static ca.mcgill.ecse211.lab1.Resources.BAND_WIDTH;
import static ca.mcgill.ecse211.lab1.Resources.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab1.Resources.MOTOR_HIGH;
import static ca.mcgill.ecse211.lab1.Resources.MOTOR_LOW;
import static ca.mcgill.ecse211.lab1.Resources.RIGHT_MOTOR;

public class BangBangController extends UltrasonicController {

  private static int distError;
  
  public BangBangController() {
    LEFT_MOTOR.setSpeed(MOTOR_HIGH);                // Start robot moving forward
    RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  @Override
  public void processUSData(int distance) {
    filter(distance);

    distError = BAND_CENTER - this.distance;        // Compute error

    if (Math.abs(distError) <= BAND_WIDTH) {        // Checks if error is within the error threshold 
      LEFT_MOTOR.setSpeed(MOTOR_HIGH);              // Start robot moving forward
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
      
    } else if (distError > 0) {                     // If it's close to the wall
      
      if (this.distance <= 5) {                     // Emergency sharp right turn used when it's really close to the wall
        LEFT_MOTOR.setSpeed(50);
        RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
        LEFT_MOTOR.backward();
        RIGHT_MOTOR.backward();
      }
      LEFT_MOTOR.setSpeed(MOTOR_HIGH);              // Right turn with right motor going backwards
      RIGHT_MOTOR.setSpeed(MOTOR_LOW);              // Designed this way since sensor is placed on the left
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
      
    } else if (distError < 0) {
      LEFT_MOTOR.setSpeed(MOTOR_LOW);               // Normal left turn
      RIGHT_MOTOR.setSpeed(MOTOR_HIGH);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
