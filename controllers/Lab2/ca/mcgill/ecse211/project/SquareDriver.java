package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class SquareDriver {

  /**
   * Drives the robot in a square of the given length. It is to be run in parallel
   * with the odometer to allow testing its functionality.
   *
   * @param length the length of the square in feet (tile sizes)
   */
  public static void driveInASquare(double length) {
    setAcceleration(ACCELERATION);
    for (int i = 0; i < 4; i++) {
      moveStraightFor(length);
      turnBy(90.0); // degrees clockwise
    }
    stopMotors();
  }
  
  /**
   * Moves the robot straight for the given distance.
   *
   * @param distance in feet (tile sizes), may be negative
   */
  public static void moveStraightFor(double distance) {
    // TODO Set motor speeds and rotate them by the given distance.
    // This method should not return until the robot has finished moving.
    
    // TODO Set motor speeds and rotate them by the given distance.
    // This method should not return until the robot has finished moving.
    
    
    //double current_X = current_position[0];
    //double current_Y = current_position[1];
    //double current_theta = current_position[2];
    
    double distance_parcourue = 0; 

    
    
    
    if (distance > 0) {
      setSpeed(FORWARD_SPEED);

      while ( distance_parcourue != distance) {
        double[] current_position = odometer.getXyt();

        double current_X = current_position[0];
        double current_Y = current_position[1];
        // double current_theta = current_position[2];
        
        
        distance_parcourue += Math.sqrt(Math.pow(current_Y,2) + Math.pow(current_X,2));

      }
      stopMotors();
      
      
    }
    
    
    
    else { // going backward 
      setSpeed(ROTATE_SPEED);
      turnBy(180); // rotate in order for the robot to move forward 
      setSpeed(FORWARD_SPEED);
      distance = - distance; 
      while ( distance_parcourue != distance) {
        double[] current_position = odometer.getXyt();

        double current_X = current_position[0];
        double current_Y = current_position[1];
        // double current_theta = current_position[2];
        
        
        distance_parcourue += Math.sqrt(Math.pow(current_Y,2) + Math.pow(current_X,2));

      }
      stopMotors();
      
    }
      
      
    
  
  }
  
  /**
   * Turns the robot by a specified angle. Note that this method is different from
   * {@code Navigation.turnTo()}. For example, if the robot is facing 90 degrees, calling
   * {@code turnBy(90)} will make the robot turn to 180 degrees, but calling
   * {@code Navigation.turnTo(90)} should do nothing (since the robot is already at 90 degrees).
   *
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    // TODO Hint: similar to moveStraightFor(), but use a minus sign
    setSpeed(ROTATE_SPEED);

    // double angle_rotated = 0 ; 

    /*while (angle_rotated!=angle) {
      double[] current_position = odometer.getXyt();


      double current_theta = current_position[2];
      angle_rotated+=current_theta; */

    leftMotor.rotate(-convertAngle(angle), true);
    rightMotor.rotate(convertAngle(angle), false);
  }
  
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   *
   * @param distance the input distance in meters
   * @return the wheel rotations necessary to cover the distance in degrees
   */
  public static int convertDistance(double distance) {
    // TODO Compute and return the correct value.
    int converted_distance = (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD)) ;
    return converted_distance;
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate the robot by that
   * angle.
   *
   * @param angle the input angle in degrees
   * @return the wheel rotations necessary to rotate the robot by the angle in degrees
   */
  public static int convertAngle(double angle) {
    // TODO Compute and return the correct value. Hint: you can reuse convertDistance()
    int converted_angle = convertDistance(angle*Math.PI*BASE_WIDTH /360.0) ;
    return converted_angle;
  }
  
  /**
   * Stops both motors.
   */
  public static void stopMotors() {
    leftMotor.stop();
    rightMotor.stop();
  }
  
  /**
   * Sets the speed of both motors to the same values.
   *
   * @param speed the speed in degrees per second
   */
  public static void setSpeed(int speed) {
    // TODO Implement this by reusing an existing method (1 line)
    setSpeeds(speed,speed);

  }
  
  /**
   * Sets the speed of both motors to different values.
   *
   * @param leftSpeed the speed of the left motor in degrees per second
   * @param rightSpeed the speed of the right motor in degrees per second
   */
  public static void setSpeeds(int leftSpeed, int rightSpeed) {
    // TODO
    leftMotor.setSpeed(leftSpeed); //change left motor speed
    rightMotor.setSpeed(rightSpeed); //change right motor speed
  }
  
  /**
   * Sets the acceleration of both motors.
   *
   * @param acceleration the acceleration in degrees per second squared
   */
  public static void setAcceleration(int acceleration) {
    // TODO
    leftMotor.setAcceleration(acceleration); // Change the acceleration of left motor
    rightMotor.setAcceleration(acceleration); // Change the acceleration of right motor
  }

}
