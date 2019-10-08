package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Resources.*;

public class UltrasonicLocalizer implements Runnable{
   
  private static double[] angles = {0, 0};
  private float[] usData = new float[US_SENSOR.sampleSize()];
  private static double distance, deltaT;
  private final static double noise_margin = 2;
  
  public void run() {
    while (true) {
      US_SENSOR.getDistanceMode().fetchSample(usData, 0); // acquire data
      distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
      distance = distance > 150 ? 150 : distance;
      try {
        Thread.sleep(20);
      } catch (Exception e) {
    }
    }
  }
  public static void risingEdge() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    while(distance > BAND_CENTER - noise_margin) {      // Checks for the case when robot is facing a wall
      leftMotor.forward();
      rightMotor.backward();
    }
    while(distance < BAND_CENTER + noise_margin) {
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    angles[0] = odometer.getXYT()[2];
    
    
    while(distance > BAND_CENTER - noise_margin) {      // Checks for the case when robot is facing a wall
      leftMotor.backward();
      rightMotor.forward();
    }
    while (distance < BAND_CENTER + noise_margin) {     
      leftMotor.backward();
      rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    angles[1] = odometer.getXYT()[2];
    
    if (angles[0] < angles[1]) {
      deltaT = 45 - (angles[0] + angles[1]) / 2.0;
    }
    else {
      deltaT = 225 - (angles[0] + angles[1]) / 2.0;
    }
    
    double updatedHeading = odometer.getXYT()[2] + deltaT;
    updatedHeading = (updatedHeading > 360) ? updatedHeading - 360 : updatedHeading;
    odometer.setXYT(0, 0, updatedHeading);
    Navigation.turnTo(0);
    
    leftMotor.stop(true);
    rightMotor.stop(false);
  }
  
  public static void fallingEdge() {
    leftMotor.setSpeed(ROTATE_SPEED);
    rightMotor.setSpeed(ROTATE_SPEED);
    
    while(distance < BAND_CENTER + noise_margin) {      // Checks for the case when robot is facing a wall
      leftMotor.forward();
      rightMotor.backward();
    }
    while(distance > BAND_CENTER - noise_margin) {
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    angles[0] = odometer.getXYT()[2];
    
    
    while(distance < BAND_CENTER + noise_margin) {      // Checks for the case when robot is facing a wall
      leftMotor.backward();
      rightMotor.forward();
    }
    while (distance > BAND_CENTER - noise_margin) {     
      leftMotor.backward();
      rightMotor.forward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    angles[1] = odometer.getXYT()[2];
    
    if (angles[0] > angles[1]) {
      deltaT = 45 - (angles[0] + angles[1]) / 2.0;
    }
    else {
      deltaT = 225 - (angles[0] + angles[1]) / 2.0;
    }
    
    double updatedHeading = odometer.getXYT()[2] + deltaT;
    updatedHeading = (updatedHeading > 360) ? updatedHeading - 360 : updatedHeading;
    odometer.setXYT(0, 0, updatedHeading);
    Navigation.turnTo(0);

    leftMotor.stop(true);
    rightMotor.stop(false);
  }
  
 
  
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }
  
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * TRACK * angle / 360.0);
  }
  
  public static double getDistance() {
    return distance;
  }
}
