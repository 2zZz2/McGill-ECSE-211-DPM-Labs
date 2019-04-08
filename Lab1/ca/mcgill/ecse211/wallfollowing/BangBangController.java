package ca.mcgill.ecse211.wallfollowing;
import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int motorLow;
  private final int motorHigh; 
  private static int BAND_WIDTH=5; //5 different from band center in P Controller
  private static final int SLEEP_INT=50; //used when we catch exceptions
  private static final int FILTER_OUT =30; //new field added for our filter
  private int filterControl=0; //initial value for the counter of our filter
  private int distance;
  
  
  
  public BangBangController(int bandCenter, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = 30; //=bandCenter
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  
  
  @Override
  public void processUSData(int distance) {
    this.distance = distance;
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
    
    
    //firstly, filter out the useless data
    if (distance >= 100 && filterControl < FILTER_OUT) {
        // bad value, filter out
        filterControl++;
        return;
      } else if (distance >= 100) {
        // We have repeated large values, leave the distance alone
        this.distance = distance;
      } else {
        // distance went below 255: reset filter and leave distance alone.
        filterControl = 0;
        this.distance = distance;     
      }
   
       
    
  //secondly, compute error
    int errorDist = 0;
    errorDist = bandCenter - distance; //negative means too far; positive means too close
    
    
    
  //thirdly, tell which condition the car is under and react correspondingly to the condition
    if (Math.abs(errorDist) <= BAND_WIDTH) { //Error within bandwidth, we don't have to turn left or right	
    		WallFollowingLab.leftMotor.setSpeed (motorHigh); //Start moving forward
    		WallFollowingLab.rightMotor.setSpeed(motorHigh);
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();
    } else if (errorDist > 5) { //when the car is too close to the wall  	
    		WallFollowingLab.leftMotor.setSpeed (260); 
    		WallFollowingLab.rightMotor.setSpeed(-30); 
    		WallFollowingLab.leftMotor.forward();
    		WallFollowingLab.rightMotor.forward();  
    } else if (errorDist < 0) { //turn left when the car is too far from the wall 	
    		WallFollowingLab.leftMotor.setSpeed (motorLow-10); 
    		WallFollowingLab.rightMotor.setSpeed(motorHigh); 
    		WallFollowingLab.leftMotor.forward(); 
    		WallFollowingLab.rightMotor.forward();
    }
 
    
    
    //fourthly, catch the exceptions
    try {
		Thread.sleep(SLEEP_INT);
	} catch (InterruptedException e) {
		e.printStackTrace();
	}
       
    
  }
    

  @Override
  public int readUSDistance() {
    return this.distance;
  }
  

  
  
}
