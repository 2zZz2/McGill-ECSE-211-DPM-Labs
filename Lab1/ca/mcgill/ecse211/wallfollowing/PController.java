package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private static final double PCOEFFICIENT = 8.5; //our coefficient for the p controller
  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  
  
  
  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }


  @Override
  public void processUSData(int distance) {

	  
	  
    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 100 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >=100) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    
    
    // TODO: process a movement based on the us distance passed in (P style)

    
    
    //calculate the error distant and delta speed
    int errorDist = bandCenter + 5 - distance; //negative means too far; positive means too close   
    int deltaSpeed = (int) Math.abs(errorDist * PCOEFFICIENT); 
    if(deltaSpeed>130) deltaSpeed = 130; //set a highest boundary to delta speed when the distance is far
    
    
    
    /* set the motor speed under different condition
     * using the delta speed we just calculate
     * 3 conditions: within interval, far from wall, close to wall*/      
    if (Math.abs(errorDist) <= bandWidth ) { //if distance is within the interval, move forward with speed proportional to the distance
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); //both motor speed are the same relative to each other
    		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); //do not turn left or turn right
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    } else if (errorDist < -2) { //turn left when the distance is too far
    		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED/2); 
    		WallFollowingLab.rightMotor.setSpeed((MOTOR_SPEED + deltaSpeed-10)/2); //we want to make the left turn small so that it won't touch the wall
    		WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    } else if (errorDist > 0) { //turn right when the car is too close to the wall  
    		if(distance>16) { //turn right when the car is close but not too close to the wall
    			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED + deltaSpeed*2);
        		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
        		WallFollowingLab.leftMotor.forward();
        		WallFollowingLab.rightMotor.forward();
        	} else { //when the distance is really close	
        		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+deltaSpeed*2);
        		WallFollowingLab.rightMotor.setSpeed(deltaSpeed);
    			WallFollowingLab.rightMotor.backward();
    			WallFollowingLab.leftMotor.forward();
        	}
    }
  }

  
  

  @Override
  public int readUSDistance() {
    return this.distance;
  }


}

