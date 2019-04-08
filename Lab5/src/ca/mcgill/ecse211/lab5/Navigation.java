package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is used to drive the robot on the demo floor.
 */
public class Navigation{
  //private Odometer odometer;
 
  //private Odometer odometer;
  private static final int FORWARD_SPEED = 180;
  public static final String[] colors = {"red","green","blue","yellow"};
  private int targetColour;
  private char planningType;
  private int LLx;
  private boolean firstSide;
  private int URx;
  private int LLy;
  private int URy;
  public boolean obstacle;
  public static final double tileSize = 30.48;
  private static final int ROTATE_SPEED = 100;
  private static final double TILE_SIZE = 30.48;
  public static Odometer odometer;
  private static EV3LargeRegulatedMotor leftMotor ;
  private static EV3LargeRegulatedMotor rightMotor;
  private ColorClassification colorDetector;
  static final double leftRadius = 2.10;
  static final double rightRadius = 2.10;
  static final double track = 13.6;
  private static final int safeDistance= 8;
  private SampleProvider gyAngles;
  private SampleProvider us;
  private float[] usValues;
  private float[] angles; 
  private boolean travelling;
  private LightLocalizer ll;
  private boolean planning;
  private boolean found;
   static double jobx;
   static double joby;
   public boolean search;
  private static boolean avoid=false;
  private static boolean back = true;
  private static boolean justAvoid = false;
  
  
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public Navigation(EV3LargeRegulatedMotor leftmotor,EV3LargeRegulatedMotor rightmotor,int targetColour,int URx,int LLx,int URy,int LLy,ColorClassification colorDetector,SampleProvider gyAngle,float[] angles,SampleProvider us, float[] usValues,LightLocalizer ll) throws OdometerExceptions {
	this.LLx = LLx;
	this.LLy = LLy;
	this.URx = URx;
	this.URy = URy;
	this.us = us;
	this.usValues = usValues;
	this.angles = angles;
	this.gyAngles = gyAngle;
	this.planning = false;
	this.targetColour = targetColour;
	this.obstacle = false;
	this.found = false;
	this.colorDetector = colorDetector;
	this.search = false;
	this.travelling = false;
	leftMotor = leftmotor;
    rightMotor = rightmotor;
    jobx = 0;
    joby = 0;
    this.ll = ll;
  }

  /**
   * This method is meant to drive the robot in a square of size 2x2 Tiles. It is to run in parallel
   * with the odometer and Odometer correction classes allow testing their functionality.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param leftRadius
   * @param rightRadius
   * @param width
   */
  public void travelTo(double x,double y ) throws OdometerExceptions{
	  System.out.println("aim at "+ x + " " + y);
	  if (!planning) {
	    	double distance1 = Math.hypot(odometer.getXYT()[0]-x, odometer.getXYT()[1]-y);
	    	if(distance1 < 3) {
	    		System.out.println(" here");
	    		return;
	    	}
	    } 
	jobx = x;
	joby = y;
	double theta, dX, dY, distance;
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.stop();
      motor.setAcceleration(3000);
    }

    // Sleep for 2 seconds
    try {
    	
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
    System.out.println("aim at "+ x + " " + y);
    Sound.beepSequenceUp();
    //TO DO SEE POSITION TO NEXT POSITION 
   // angle = odometer.getXYT()[2];
    
    dX = x - odometer.getXYT()[0];
    dY = y - odometer.getXYT()[1];
    double oldx = odometer.getXYT()[0];
    double oldy = odometer.getXYT()[1];
    distance = Math.sqrt(dX*dX + dY*dY);
    theta = Math.toDegrees(Math.atan(dX/dY));
    
    if(dX < 0 && dY < 0) {
    	theta += 180;
    }
    if(dX > 0 && dY < 0) {
    	theta += 180;
    }
    if(dX < 0 && dY > 0) {
    	theta += 360;
    }
    turnTo(theta);
    double correctAngle = gyroFetch();
    if (Math.abs(correctAngle - theta)>1.5) {
    	angleCorrection();
    	if (Math.min(Math.abs(correctAngle - theta),(360-Math.abs(correctAngle - theta)))>8) {
    		System.out.println("angle error "+ correctAngle + theta);
    		travelTo(jobx,joby);
    		return;
    	}
    }
    this.travelling = true;
   if(search) {
    leftMotor.setSpeed((int)(FORWARD_SPEED*0.8));
    rightMotor.setSpeed((int)(FORWARD_SPEED*0.8));
   }else {
	   leftMotor.setSpeed(FORWARD_SPEED);
	    rightMotor.setSpeed(FORWARD_SPEED); 
   }
   leftMotor.forward();
   rightMotor.forward();
   // leftMotor.rotate(convertDistance(leftRadius, distance), true);
   // rightMotor.rotate(convertDistance(rightRadius, distance), true);
    while(true) {
    	//if(obstacle) {
    	directionCorrection(correctAngle,FORWARD_SPEED);
    	if(usFetch() < 13) {
    		avoid(usFetch());
    		back = true;
    		if(planning) {
    		if(found) {
    			planning = false;
    			travelTo(URx*tileSize,URy*tileSize);
    			System.exit(0);
    		}
    		}
    		if (!planning) {
    			travelTo(jobx,joby);   
    		}
    		break;
    	}
    if (!planning) {
    	double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	if(distance1 < 3) {
    		break;
    	}
    	if (losePath(oldx,oldy)) {
    		travelTo(jobx,joby);
    	}
    } else {
    				
    			
    			double distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	if(distance1 > 0.5) {
    	    		distance1 = Math.hypot(odometer.getXYT()[0]-jobx, odometer.getXYT()[1]-joby);
    	    	}else {
    	    		break;
    	    	}
    	    	if (losePath(oldx,oldy)) {
    	    		travelTo(jobx,joby);
    	    	}
    		}

    }
    carStop();
    travelling = false;
   
  }
  /**
   * this method is used to call the color classification and detect the color of the can and avoid the can during the 
   * travelling state 
   * @param distance
   * @throws OdometerExceptions
   */
  public  void avoid(double distance) throws OdometerExceptions{
	int color = 4;
    leftMotor.stop(true);
	rightMotor.stop();
	if(planning) {
		leftMotor.setSpeed(FORWARD_SPEED/3);
	    rightMotor.setSpeed(FORWARD_SPEED/3);
	    leftMotor.rotate(convertDistance(leftRadius, 7), true);
	    rightMotor.rotate(convertDistance(rightRadius, 7), false);
	    carStop();
	  color = colorDetector.findColor();
	  if (color == targetColour) {
	    Sound.beep();
	    System.out.println(odometer.getXYT()[0]+" "+odometer.getXYT()[1]+" "+odometer.getXYT()[2]);
	    this.found = true;
	    this.planning = false;
	    travelTo(URx*tileSize,URy*tileSize);
 	    System.exit(0);
	    return;
	  }else {
	    Sound.twoBeeps();
	  }
	  leftMotor.setSpeed(FORWARD_SPEED/3);
	  rightMotor.setSpeed(FORWARD_SPEED/3);
	  leftMotor.rotate(-convertDistance(leftRadius, 7), true);
	  rightMotor.rotate(-convertDistance(rightRadius, 7), false);
	  carStop();
	  justAvoid = true;
	}
	travelling = false;
	  if (predictPath() == 1){
	    RightAvoid(color);
	  }
	  else if (predictPath() == 0){
	    leftAvoid(color);
	  }
	  if (found) {
			return;
		}
	
	this.obstacle = false;
	   
  }
  
  

  /**
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  public void turnTo (double theta) {
	  double angle,smallestAngle;
	 // angle = odometer.getXYT()[2];
	  angle = gyroFetch();
	    if((theta - angle) > 180) {
	    	smallestAngle = theta - angle - 360;
	    	//turnLeft(360-(theta-angle));
	    }
	    else if((theta - angle) < -180) {
	    	smallestAngle = theta - angle + 360;
	    	//turnRight(360 -(angle-theta));
	    }
	    else {
	    	smallestAngle = theta - angle;
	    	//turnRight(smallestAngle);
	    }
	        // turn 90 degrees clockwise
	        leftMotor.setSpeed(ROTATE_SPEED);
	        rightMotor.setSpeed(ROTATE_SPEED);

	        leftMotor.rotate(convertAngle(leftRadius, track, smallestAngle), true);
	        rightMotor.rotate(-convertAngle(rightRadius, track, smallestAngle), false);
  }
   boolean  isNavigating(){
	  return this.travelling;
  }
   /**
    * this method is used to determine the turning direction depends on the position of the car,call by avoid function
    * @return the turning direction 1 means right ,0 means left
    */

  public int predictPath() {

		double currx = odometer.getXYT()[0];
		double curry = odometer.getXYT()[1];
		double currTheta = odometer.getXYT()[2];
		
		if (currTheta > 340 || currTheta <= 20) {//going up
			if (currx < (LLx+0.5)*tileSize) {
				return 1;
				//wallFollowRight();            // 1 represents right dodge and 0 represents left dodge
			} 
			else if (currx > (URx-0.5)*tileSize){
				return 0;
				//wallFollowLeft();
			}
		} 
		else if(currTheta >= 70 && currTheta < 110){//going right
			if (curry < (LLy+0.5)*tileSize) {
				return 0;
				//wallFollowLeft();
			} 
			else if (curry > (URy-0.5)*tileSize) {
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 160 && currTheta < 200){//going down
			if (currx < (LLx+0.5)*tileSize) {
				return 0;
				//wallFollowLeft();
			} 
			else if (currx > (URx-0.5)*tileSize) {
				return 1;
				//wallFollowRight();
			}
		}
		else if(currTheta > 250 && currTheta < 290){//going left
			if (curry <= (LLy+0.5)*tileSize ) {
				return 1;
				//wallFollowRight();
			} 
			else if (curry > (URy-0.5)*tileSize) {
				System.out.println("8");
				return 0;
				//wallFollowLeft();
			}
		}
			//wallFollowRight();
			//return 1;
			if (this.planningType == 'V') {
				if (this.firstSide) {
					return 0;
				}else {
					return 1;
				}
			}else if (this.planningType == 'H'){
				if(this.firstSide) {
					return 1;
			}else {
				return 0;
			}
				
			}else {
				return 0;
			}
  
	
	}
  /**
   * the method help the car turn right,and determine whether need to test the color again,called by avoid function
   * @param color
   */
  public void RightAvoid(int color) throws OdometerExceptions {
	  avoid = true;
	  back =false;
	  carStop();
	  turnRight(90);
	  goStraightLine(15,FORWARD_SPEED/2);
      turnLeft(90);
      goStraightLine(18,FORWARD_SPEED/2);
      if (color == -1) {
    	  turnLeft(90);
    	  rightTest();
  	      carStop();
          if (search && !endOfSearch()) {
        	  turnRight(90);
        	  goStraightLine(15,FORWARD_SPEED/2);
              carStop();
              turnLeft(90);
              goStraightLine(15,FORWARD_SPEED/2);
              carStop();
              turnRight(90);
              back = true;
          }
      }else {
    	  if (search && !endOfSearch()) {
    		  goStraightLine(15,FORWARD_SPEED/2);
              turnLeft(90);
              goStraightLine(15,FORWARD_SPEED/2);
              carStop();
              back = true;
    	  }
      }
      avoid = false;
      
	  
	}
  /**
   * the method help the car turn right,and determine whether need to test the color again,called by avoid function
   * @param color
   */
	
	public void leftAvoid (int color) throws OdometerExceptions {
		back  = false;
		avoid = true;
		carStop();
		  turnLeft(90);
	      carStop();
	      gyroFetch();
		  goStraightLine(18,FORWARD_SPEED/2);
	      turnRight(90);
	      carStop();
	      gyroFetch();
	      goStraightLine(18,FORWARD_SPEED/2);
	      carStop();
	      if (color == -1) {
	    	  leftTest();
	  	      gyroFetch();
	  	      carStop();
	          if (search && !endOfSearch()) {
	        	  turnLeft(90);
	        	  goStraightLine(18,FORWARD_SPEED/2);
	              carStop();
	              gyroFetch();
	              turnRight(90);
	              goStraightLine(17,FORWARD_SPEED/2);
	              carStop();
	              gyroFetch();
	         
	              carStop();
	              turnLeft(90);
	              back = true;
	          }
	      }else {
	    	  if (search && !endOfSearch() ) {
	    		  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              gyroFetch();
	              turnRight(90);
	              goStraightLine(17,FORWARD_SPEED/2);
	              carStop();
	              turnLeft(90);
	              back = true;
	    	  }
		avoid = false;
	}
	}
/**
 * calculate the angle that wheel need to rotate to make car move forward a certain distance
 * @param radius
 * @param distance
 * @return the angle that wheel need to rotate
 */

  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
/**
 * calculate the angle that wheel need to rotate to make car turning a certain angle
 * @param radius
 * @param width
 * @param angle
 * @return
 */
 
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  /**
   * planning the path which contain the point that car will go through that the car will move if (URy - LLy) > (URx - LLx)
   * @param Rx
   * @param Ry
   * @throws OdometerExceptions
   */
  public void verticalPlanning(int Rx,int Ry) throws OdometerExceptions {
	  this.planningType = 'V';
	  this.firstSide = true;
	  this.planning = true;
	  for(int i = 0;i <= Ry;i++) {
		  
		  if(this.firstSide) {
			  turnTo(90);
		  }else {
			  turnTo(270);
		  }
		  if (usFetch()<(Rx*tileSize)) {
			  Sound.buzz();
			  back = true;
			  this.search = true;
			  if(this.firstSide) {
				  for(int j = 1;j<=Rx;j++) {// search process
					  justAvoid = false;
				      travelTo((this.LLx+j)*tileSize,(this.LLy+i)*tileSize);
				      carStop();
				      if (!justAvoid) {
				    	Sound.beepSequence();
					    search();
				  }
				  }
			  }else {
				  for(int j = 1;j<=Rx;j++) {// search process
					  justAvoid = false;
				      travelTo((this.URx-j)*tileSize,(this.LLy+i)*tileSize);
				      carStop();
				      if (!justAvoid) {
				    	  Sound.beepSequence();
						  search();
					  }
				  }
			  }
			 this.firstSide = !this.firstSide;
		  }
		  if (!back) {
			  if (this.firstSide) {
				  turnLeft(90);
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnLeft(90);
	              goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnRight(90);
			  }else {
				  turnRight(90);
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnRight(90);
	              goStraightLine(15,FORWARD_SPEED/2);
	              turnLeft(90);
			  }
		  }
		 this.search = false; 
		  if (this.found) {
			  break;
		  }
		  if(i < Ry) {
			    if(this.firstSide) {
			    	justAvoid = false;
			      travelTo(this.LLx*tileSize,(this.LLy+i+1)*tileSize);
			      if (!justAvoid) {
				    	Sound.beepSequence();
					    search();
				  }
			      
			    }else {
			    	justAvoid = false;
			    	travelTo(this.URx*tileSize,(this.LLy+i+1)*tileSize);
			    	if (!justAvoid) {
				    	Sound.beepSequence();
					    search();
				  }
			    }
			  }
	  }
	  planning = false;
	  travelTo(URx*tileSize,URy*tileSize);
  }
  /**
   * planning the path which contain the point that car will go through that the car will move if (URx - LLx) > (URy - LLy)
   * @param Rx
   * @param Ry
   * @throws OdometerExceptions
   */
  public void horizontalPlanning(int Rx,int Ry) throws OdometerExceptions {
	  this.planningType = 'H';
	   this.firstSide = true;
	  this.planning = true;
	  for(int i = 0;i <= Rx;i++) {
		  if(this.firstSide) {
			  turnTo(0);
		  }else {
			  turnTo(180);
		  }
		  if ((usFetch())<(Rx*tileSize)) {
			  back = true;
			  this.search = true;
			  if(this.firstSide) {
				  for(int j = 1;j<=Ry;j++) {// search process
					justAvoid = false;  
				    travelTo((this.LLx+i)*tileSize,(this.LLy+j)*tileSize);
				    carStop();
				    if (!justAvoid) {
						  search();
					  }
				  }
			  }else {
				  for(int j = 1;j<=Ry;j++) {// search process
					justAvoid = false;  
				    travelTo((this.LLx+i)*tileSize,(this.URy-j)*tileSize);
				    carStop();
				    if (!justAvoid) {
					  search();
				  }
			  }
			  }
			  this.firstSide = !this.firstSide;
		  }
		  if (!back) {
			  if (this.firstSide) {
				  turnRight(90);
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnRight(90);
	              goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnLeft(90);
			  }else {
				  turnLeft(90);
				  goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnLeft(90);
	              goStraightLine(15,FORWARD_SPEED/2);
	              carStop();
	              turnRight(90);
			  }
		  }
		  this.search = false;
		  if (this.found) {
			  break;
		  }
		  if(i < Rx) {
			    if(this.firstSide) {
			    	justAvoid = false;
			      travelTo((this.LLx+i+1)*tileSize,this.LLy*tileSize);
			      if (!justAvoid) {
				    	Sound.beepSequence();
					    search();
				  }
			    }else {
			    	travelTo((this.LLx+i+1)*tileSize,this.URy*tileSize);
			    	if (!justAvoid) {
				    	Sound.beepSequence();
					    search();
				  }
			    }
			  }
	  
	  }
	  planning = false;
	  travelTo(URx*tileSize,URy*tileSize);
	  
  }
  /**
   * the whole process that the car will do after the localization
   * @throws OdometerExceptions
   */
  public void work() throws OdometerExceptions {
	  System.out.println(LLx+" "+LLy);
	  travelTo(tileSize,tileSize);
	  turnTo(0);
	  leftMotor.rotate(-convertAngle(leftRadius, track, 4), true);
      rightMotor.rotate(convertAngle(rightRadius, track, 4), false);
      Lab5.gySensor.reset();
      angleCorrection();
	  if((LLx + LLy)>5) {
      travelTo((LLx-0.3) * tileSize,(LLy-0.3)*tileSize);
      turnTo(0);
	  ll.doLocalizationStart();
	  }else {
		  ll.closeSensor();
	  }
	  
	  travelTo(LLx*tileSize,LLy*tileSize);
	  Sound.beep();
	  turnTo(0);
	  int Rx = this.URx - this.LLx;
	  int Ry = this.URy - this.LLy;
	  if (Rx <= Ry) {
		  try {
		  verticalPlanning(Rx,Ry);
		  }catch (OdometerExceptions e) {
				
				e.printStackTrace();
			}
	  }
	  else {
		  try {
		  horizontalPlanning(Rx,Ry);
		  }catch (OdometerExceptions e) {
				
				e.printStackTrace();
			}
	  }
  }
  /**
   * get the distance value from the ultrasonic sensor
   * @return the distance value read by ultrasonic sensor
   */
  private float usFetch() {
	  us.fetchSample(usValues, 0);
	  if ((usValues[0]*100)>200) {
		  return 200;
	  }else {
		  return usValues[0]*100;
	  }
  }
  /**
   * get the angular value from the  gyrosensor
   * @return the angular value measure by gyrosensor 
   */
  private double gyroFetch() {
	  this.gyAngles.fetchSample(angles, 0);
	  angleCorrection();
	  return odometer.getXYT()[2];
  }
  /**
   * use the gyro sensor to correct the angle value in the odometer
   */
  private void angleCorrection() {
	  gyAngles.fetchSample(angles, 0);
	  if (angles[0] >= 0) {
		  odometer.setXYT(odometer.getXYT()[0], odometer.getXYT()[1],angles[0]);
	  }else {
		  odometer.setXYT(odometer.getXYT()[0], odometer.getXYT()[1], 360+angles[0]);
	  }
  }
  /**
   * use the sensor the check whether the car go a straight line,and modified the direction if there's a deviation
   * @param degree
   * @param speed
   */
  private void directionCorrection(double degree,int speed) {
	    double degree1 = gyroFetch();
		if(degree1 - degree  >= 2) {
			leftMotor.setSpeed(speed);
	        rightMotor.setSpeed((float)(speed+(degree1 - degree)*6));
	        leftMotor.forward();
  		rightMotor.forward();
	this.gyAngles.fetchSample(angles, 0);
  }else if(degree1 - degree <=-2) {
  		leftMotor.setSpeed((float)(speed+(degree - degree1)*6));
	        rightMotor.setSpeed(speed);
	        leftMotor.forward();
  		rightMotor.forward();
	this.gyAngles.fetchSample(angles, 0);
  	}
  }
  /**
   * let car move forward a certain street and use the sensor to do the direction correction
   * @param distance
   * @param speed
   */
  private void goStraightLine(double distance,int speed) {
	  carStop();
	  double angle = gyroFetch();
	  double p1 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
	  double p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
	 double distance1 = p2 - p1;
	int speed1;
	while(distance1 < distance) {
		speed1 =(int) (speed - 25/((distance - distance1)+1));
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		leftMotor.forward();
		rightMotor.forward();
		for(int i = 0;i<3;i++);
		directionCorrection(angle,speed1);
		p2 = (leftMotor.getTachoCount()+rightMotor.getTachoCount())*leftRadius*Math.PI/360;
		distance1 = p2 - p1;
	}
	  carStop();
  }
/**
 * make the car turning right
 * @param degree
 */
  private void turnRight(double degree) {
	  if(degree <= 1) {
		  return;
	  }
	  double minAngle = 0;
	    int speed;
		 double angle = gyroFetch();
		 double angle1 = gyroFetch();
		 while((Math.abs(angle1 - angle - degree)>=1) && (Math.abs((angle - angle1) - (360-degree))>=1)){
			 minAngle = Math.min((Math.abs(angle1 - angle - degree)), Math.abs((angle - angle1) - (360-degree)));
			 speed = (int)(80 - 25/(minAngle+1));
			 leftMotor.setSpeed(speed);
		     rightMotor.setSpeed(speed);
		     leftMotor.forward();
		     rightMotor.backward();
		     angle1 = gyroFetch();
		 }
			leftMotor.stop(true);
			rightMotor.stop();
		}
  /**
   * make the car turning left
   * @param degree
   */
		private void turnLeft(double degree) {
			if (degree <= 1) {
				return;
			}
			int speed;
			double minAngle = 0;
			 double angle = gyroFetch();
			 double angle1 = gyroFetch();
			 while((Math.abs(angle - angle1 - degree)>=1) && (Math.abs((angle1 - angle) - (360-degree))>=1)){
				 minAngle = Math.min((Math.abs(angle - angle1 - degree)), Math.abs((angle1 - angle) - (360-degree)));
				 speed = (int)(80 - 25/(minAngle+1));
				 leftMotor.setSpeed(speed);
			     rightMotor.setSpeed(speed);
			     leftMotor.backward();
			     rightMotor.forward();
			     angle1 = gyroFetch();
			 }
				leftMotor.stop(true);
				rightMotor.stop();
			}
		/**
		 * check if the car is in the last point of the search process
		 * @return
		 */
 private boolean endOfSearch() {
	 if (planningType == 'H') {
		 if(firstSide) {
			 return (Math.abs(joby - URy*tileSize) <= 1);
		 }else {
			 return (Math.abs(joby - LLy*tileSize) <= 1);
		 }
	 }else {
		 if(firstSide) {
			 return (Math.abs(jobx - URx*tileSize) <= 1);
		 }else {
			 return (Math.abs(jobx - LLx*tileSize) <= 1);
		 }
	 }
 }
 /**
  * check if car is far from the destination after travelTo function if the answer is true ,the travelTo function will be call again
  * @param x
  * @param y
  * @return
  */
 private boolean losePath(double x,double y) {
	 double curx = odometer.getXYT()[0];
	 double cury = odometer.getXYT()[1];
	// System.out.println(curx+" "+cury);
	 boolean x1=false,y1=false;
	 if (jobx < x && curx < x) {
		 x1 = (jobx - curx >= 6.5);
	 }else if (jobx > x && curx > x) {
		 x1 = ((curx - jobx) >= 6.5);
	 }else if ((jobx > x && curx < x)||(jobx < x && curx > x)) {
		 x1 = (Math.abs(curx - x) >= 6.5);
	 }
	 if (joby < y && cury < y) {
		 y1 = (joby - cury >= 6.5);
	 }else if (joby > y && cury > y) {
		 y1 = ((cury - joby) >= 6.5);
	 }else if ((joby > y && cury < y)||(joby < y && cury > y)) {
		 y1 = Math.abs(cury - y) >= 4;
	 }
	 if (x1||y1) {
		 System.out.println(x1);
		 System.out.println(y1);
		 System.out.println("lose");
	 }
	 return x1||y1;
 }
 /**
  * make the car stop
  */
  private void carStop() {
	    leftMotor.stop(true);
	    rightMotor.stop();
	  }
  /**
   * in the search process, check if there's a can near the car,make sure there's no missing can 
   * @throws OdometerExceptions
   */
  private void search() throws OdometerExceptions {
	  double angle = gyroFetch();
	  double curAngle = gyroFetch();
	  double difAngle = 0;
	  while(usFetch()>10 && difAngle < 300) {
		  leftMotor.setSpeed(70);
		  rightMotor.setSpeed(70);
		  leftMotor.forward();
		  rightMotor.backward();
		  curAngle = gyroFetch();
		  if (angle > 180 && curAngle < angle) {
			  difAngle = 360 - (angle - curAngle);
		  }else if (angle < 180 && curAngle > angle) {
			  difAngle = curAngle -angle;
		  }else if (angle <= 180 && curAngle < angle) {
			  difAngle = 360 - (angle - curAngle);
		  }
	  }
	  carStop();
	  double distance = usFetch();
	  if (distance<=10) {
		  leftMotor.setSpeed(45);
		  rightMotor.setSpeed(45);
		  if (distance > 6) {
		  leftMotor.rotate(convertDistance(leftRadius, usFetch()-6), true);
  	      rightMotor.rotate(convertDistance(rightRadius, usFetch()-6), false);
		  }else {
			  leftMotor.rotate(-convertDistance(leftRadius, 6-usFetch()), true);
	  	      rightMotor.rotate(-convertDistance(rightRadius, 6-usFetch()), false);
		  }
		  int color = colorDetector.findColor();
		  if (color == targetColour) {
		    Sound.beep();
		    System.out.println(odometer.getXYT()[0]+" "+odometer.getXYT()[1]+" "+odometer.getXYT()[2]);
		    this.found = true;
		    this.planning = false;
		    travelTo(URx*tileSize,URy*tileSize);
		    System.exit(0);
		  }else {
		    Sound.twoBeeps();
		  }
	  }else {
		  return;
	  }
  }
  /**
   * test the car again if the first classification is not successful,call by LeftAvoid method
   * @throws OdometerExceptions
   */
 private void leftTest() throws OdometerExceptions {
	 while(usFetch()>10) {
		 leftMotor.setSpeed(50);
		 rightMotor.setSpeed(50);
		 leftMotor.forward();
		 rightMotor.backward();
	 }
	 carStop();
	 leftMotor.rotate(convertAngle(leftRadius, track, 5), true);
     rightMotor.rotate(-convertAngle(rightRadius, track, 5), false);
     carStop();
	 leftMotor.setSpeed(FORWARD_SPEED/3);
	 rightMotor.setSpeed(FORWARD_SPEED/3);
	 leftMotor.rotate(convertDistance(leftRadius, 4), true);
	 rightMotor.rotate(convertDistance(rightRadius, 4), false);
	 carStop();
	 int color = colorDetector.findColor();
	  if (color == targetColour) {
	    Sound.beep();
	    System.out.println(odometer.getXYT()[0]+" "+odometer.getXYT()[1]+" "+odometer.getXYT()[2]);
	    this.found = true;
	    this.planning = false;
	    travelTo(URx*tileSize,URy*tileSize);
	    System.exit(0);
	  }else {
	    Sound.twoBeeps();
	  }
	  leftMotor.setSpeed(FORWARD_SPEED/3);
	  rightMotor.setSpeed(FORWARD_SPEED/3);
	  leftMotor.rotate(-convertDistance(leftRadius, 4), true);
	  rightMotor.rotate(-convertDistance(rightRadius, 4), false);
	  carStop();
 }
 /**
  * test the car again if the first classification is not successful,call by LeftAvoid method
  * @throws OdometerExceptions
  */
 private void rightTest() throws OdometerExceptions {
	 while(usFetch()>10) {
		 leftMotor.setSpeed(50);
		 rightMotor.setSpeed(50);
		 leftMotor.backward();
		 rightMotor.forward();
	 }
	 carStop();
	 leftMotor.rotate(-convertAngle(leftRadius, track, 5), true);
     rightMotor.rotate(convertAngle(rightRadius, track, 5), false);
     carStop();
	 leftMotor.setSpeed(FORWARD_SPEED/3);
	 rightMotor.setSpeed(FORWARD_SPEED/3);
	 leftMotor.rotate(convertDistance(leftRadius, 4), true);
	 rightMotor.rotate(convertDistance(rightRadius, 4), false);
	 int color = colorDetector.findColor();
	  if (color == targetColour) {
	    Sound.beep();
	    System.out.println(odometer.getXYT()[0]+" "+odometer.getXYT()[1]+" "+odometer.getXYT()[2]);
	    this.found = true;
	    this.planning = false;
	    travelTo(URx*tileSize,URy*tileSize);
	    System.exit(0);
	  }else {
	    Sound.twoBeeps();
	  }
	  leftMotor.setSpeed(FORWARD_SPEED/3);
	  rightMotor.setSpeed(FORWARD_SPEED/3);
	  leftMotor.rotate(-convertDistance(leftRadius,4), true);
	  rightMotor.rotate(-convertDistance(rightRadius, 4), false);
	  carStop();
 }
}
