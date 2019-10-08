package ca.mcgill.ecse211.lab3;

import static ca.mcgill.ecse211.lab3.Resources.*;

public class BangBangController extends UltrasonicController {

  private static int distError;

  @Override
  public void processUSData(int distance) {
    filter(distance);

    distError = BAND_CENTER - this.distance;       // Compute error

    if (Math.abs(distError) <= BAND_WIDTH) {       // Checks if error is within the error threshold 
      leftMotor.setSpeed(MOTOR_HIGH);              // Start robot moving forward
      rightMotor.setSpeed(MOTOR_HIGH);
      leftMotor.forward();
      rightMotor.forward();
      
    } else if (distError > 0) {                    // If it's close to the wall     
      leftMotor.setSpeed(MOTOR_HIGH);              // Right turn with right motor going backwards
      rightMotor.setSpeed(MOTOR_LOW);              // Designed this way since sensor is placed on the left
      leftMotor.forward();
      rightMotor.backward();
      
    } else if (distError < 0) {
      leftMotor.setSpeed(MOTOR_LOW);               // Normal left turn
      rightMotor.setSpeed(MOTOR_HIGH);
      leftMotor.forward();
      rightMotor.forward();
    }
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
