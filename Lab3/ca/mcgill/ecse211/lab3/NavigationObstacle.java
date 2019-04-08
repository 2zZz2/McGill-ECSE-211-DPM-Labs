package ca.mcgill.ecse211.lab3;


import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


/**
 * Navigation with obstacle 
 * 
 */
public class NavigationObstacle implements Runnable {
	
	
	/**
	 * setups for the sensor
	 * 
	 */
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private float[] usData;
	private SampleProvider usDistance ;	
	private static Odometer odometer;
	private OdometerData odoData;

	/**
	 * constants of our car & experiment place
	 */
	private static final double TILE_SIZE = 30.48;
	private static double TRACK;
	private static double WHEEL_RAD;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private static final double RIGHT_ANGLE = 90.0;//a 90 degree angle
	private static final double TURNING_DISTANCE=5;
	
	/**
	 * used in travelTo
	 */
	static double currentTheta = 0;
	double currentY;
	double currentX;
	double deltaX, deltaY, deltaT;
	double travelDist;
	int completedDestinations = 0;
	
	/**
	 * 4 possible maps
	 */
	private static final double[][] Map1 = new double[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final double[][] Map2 = new double[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final double[][] Map3 = new double[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
	private static final double[][] Map4 = new double[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
	private double[][]  destinations = Map2;
      
    /**
     * constructor
     * 
     */
	public NavigationObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		NavigationObstacle.odometer = Odometer.getOdometer();
		NavigationObstacle.leftMotor = leftMotor;
		NavigationObstacle.rightMotor = rightMotor;
		odoData = OdometerData.getOdometerData();
		odoData.setXYT(0 , 0 , 0);
		NavigationObstacle.TRACK = TRACK;
		NavigationObstacle.WHEEL_RAD = WHEEL_RAD;
		
		//setup our ultrasonic sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
		usDistance = usSensor.getMode("Distance"); 
		this.usData = new float[usDistance.sampleSize()]; 		
			
		}

		// run method (required for Thread)
	public void run() {
		
		//this make the robot run smoother
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(250);  
		}
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}
		
		
		// while loop iterates through the waypoints
		while(completedDestinations < destinations.length) {
			travelTo(destinations[completedDestinations][0], destinations[completedDestinations][1]);
			completedDestinations++;
		}
	}
	

	/**
	 * Travels to (x, y) by calculating the angle, turning towards it,
	 * then going forward until the distance between the current point and the target tolerable
	 * @param x
	 * @param y
	 */
	
	public void travelTo(double x, double y) {
		currentX = odometer.getX();// get the position on the board
		currentY = odometer.getY();
		currentTheta = odometer.getTheta();

		deltaX = x*TILE_SIZE- currentX;
		deltaY = y*TILE_SIZE - currentY;
		travelDist = Math.sqrt(deltaX*deltaX+deltaY*deltaY);
		
		
		if(deltaY>=0) {
			deltaT=Math.toDegrees(Math.atan(deltaX/deltaY));
		}
		else if(deltaY<=0&&deltaX>=0) {
			deltaT=Math.toDegrees(Math.atan(deltaX/deltaY)+Math.PI);
		}
		else {
			deltaT=Math.toDegrees(Math.atan(deltaX/deltaY)-Math.PI);
		}

		// initial angle is  0 same direction as y-axis, going clockwise
		double turningAngle = (deltaT-currentTheta); // robot has to turn "differenceInTheta",
		//turn the robot to the desired direction
		turnTo(turningAngle); 

		// drive forward by the distance needed
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, travelDist), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, travelDist), true);

		//this while loop avoid the obstacle if there is one
		while(isNavigating()) {
			usDistance.fetchSample(usData,0);
			float distance = usData[0]*100;
			
			//if distance fetched is smaller than 13, that means its too close to the obstacle, turn 90 degrees and avoid the block
			if(distance<= 13) {
				
				//turn to the left if we are running on map 3 and travel a distance
				if(odometer.getX() < 2.5*TILE_SIZE&& odometer.getX() > 1.5*TILE_SIZE&& odometer.getY() < TILE_SIZE&&odometer.getY() > 1.5*TILE_SIZE){
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), true);  
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 6*TURNING_DISTANCE), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 6*TURNING_DISTANCE), false);
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), true);
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 8*TURNING_DISTANCE), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 8*TURNING_DISTANCE), false);
				}
				//else turn right and travel a distance
				else {
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), true);   
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 6*TURNING_DISTANCE), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 6*TURNING_DISTANCE), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, RIGHT_ANGLE), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 7*TURNING_DISTANCE), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 7*TURNING_DISTANCE), false);
				}
				completedDestinations--;
			}
		}
	}
		
	/**
	 * This method causes the robot to turn (on point) to the absolute heading theta. This method should turn a MINIMAL angle to its target.
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {
		
		if(theta>180) {
			theta=360-theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
		//
		else if(theta<-180) {
			theta=360+theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
	
		else {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);	
		}
	}

	/**
	 *This method returns true if another thread has called travelTo() or turnTo() and the method has yet to return;
	 * false otherwise.
	 * 
	 */
		
	boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else 
			return false;
		}


	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param WHEEL_RAD
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method allows us to convert distance we want to travel to the number of rotation the wheels
	 * @param radius of the wheel
	 * @param distance between wheels
	 * @param angle of the turn
	 * @return number of rotations
	 */
	private static int convertAngle(double radius, double wideltaTh, double angle) {
		return convertDistance(radius, Math.PI * wideltaTh * angle / 360.0);
	}
}