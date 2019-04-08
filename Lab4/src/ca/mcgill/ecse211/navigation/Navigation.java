package ca.mcgill.ecse211.navigation;


import ca.mcgill.ecse211.localization.LightLocalizer;

public class Navigation implements Runnable {
	
	/**
	 * constants that represent properties of our car
	 */
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 100;
	private static final double TILE_SIZE = 30.48;
	private static final double WHEEL_RAD = 2.1;
	private static final double TRACK = 13.4; 
	private static final double RIGHT_ANGLE = 90.0;//a 90 degree angle
	
	
	/**
	 * 4 possible maps
	 */
	private static final int[][] Map1 = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final int[][] Map2 = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final int[][] Map3 = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
	private static final int[][] Map4 = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
	public static final int[][] destinations = Map1;
	
	
	
	/**
	 * counter for completed destinations
	 */
	public static int currentDestinations = 0;

	
	
	
	
	/**
	 * A default constructor
	 */
	public Navigation() {	
	}
	
	
	/**
	 * iterate through the destinations
	 */
	public void run() {
		for (int[] destination : destinations) {	
			travelTo(destination[0],destination[1]);
			currentDestinations++;
		}
	}
	
	
	/**
	 * Travels to (x, y) by calculating the angle, turning towards it,
	 * then going forward until the distance between the current point and the target tolerable
	 * @param x
	 * @param y
	 */
	public static void travelTo(double x, double y) {	
		
		//get current locations and headings
		double currentX=LightLocalizer.odometer.getX();
		double currentY=LightLocalizer.odometer.getY();	
		double theta=LightLocalizer.odometer.getTheta();
		
		
		//calculate the theta in the Cartesian Plan where positive y axis is pointing at 0 degree
		//positive x axis is pointing at 90; negative y axis is pointing at 180 degree; negative x axis is pointing at 360 degree
		//first quadrant
		if ((x*TILE_SIZE-currentX)>0 && (y*TILE_SIZE-currentY)>=0) theta = RIGHT_ANGLE - Math.toDegrees((Math.atan((y*TILE_SIZE-currentY)/(x*TILE_SIZE-currentX))));  
		//second quadrant
		if ((x*TILE_SIZE-currentX)<0 && (y*TILE_SIZE-currentY)>0) theta = 3*RIGHT_ANGLE - Math.toDegrees((Math.atan((y*TILE_SIZE-currentY)/(x*TILE_SIZE-currentX))));  
		//third quadrant
		if ((x*TILE_SIZE-currentX)<0 && (y*TILE_SIZE-currentY)<0) theta = 3*RIGHT_ANGLE - Math.toDegrees((Math.atan((y*TILE_SIZE-currentY)/(x*TILE_SIZE-currentX))));
		//fourth quadrant
		if ((x*TILE_SIZE-currentX)>0 && (y*TILE_SIZE-currentY)<0) theta = RIGHT_ANGLE - Math.toDegrees((Math.atan((y*TILE_SIZE-currentY)/(x*TILE_SIZE-currentX))));  
		turnTo(theta);
			
		//measure x and y again after turning
		currentX=LightLocalizer.odometer.getX();
		currentY=LightLocalizer.odometer.getY();

		//calculated the remaining distance 
		double travelDist=calculateDistance(x*TILE_SIZE, currentX, y*TILE_SIZE, currentY);	
		
		LightLocalizer.leftMotor.setSpeed(FORWARD_SPEED);
		LightLocalizer.rightMotor.setSpeed(FORWARD_SPEED);
		LightLocalizer.leftMotor.rotate(convertDistance(WHEEL_RAD, travelDist), true);
		LightLocalizer.rightMotor.rotate(convertDistance(WHEEL_RAD,travelDist), false);
		
		
	}

	
	/**
	 * The input theta is the amount in degrees of the angle to be made,
	 * assuming the robot is facing North, i.e. the absolute heading
	 * @param theta
	 */
	public static void turnTo(double theta) {
		
		double currentTheta=LightLocalizer.odometer.getTheta();
		double turningAngle=currentTheta-theta;
		
		if (currentTheta==theta) {
			//no need to perform a turn
			return;
		}	
		
		if (turningAngle>0 && turningAngle<=180) {
			//turn left
			turningAngle=currentTheta-theta;
			turnLeft(turningAngle);
			return;
		}
		
		if (turningAngle>180 && turningAngle<=360) {
			//turn right
			turningAngle=theta-currentTheta+4*RIGHT_ANGLE;
			turnRight(turningAngle);
			return;
		}
		
		if (turningAngle>-180 && turningAngle<=0) {
			//turn right
			turningAngle=theta-currentTheta;
			turnRight(turningAngle);
			return;
		}
		
		if (currentTheta-theta>-360 && currentTheta-theta<=-180) {
			//turn left
			turningAngle=currentTheta-theta+4*RIGHT_ANGLE;
			turnLeft(turningAngle);
			return;
		}
		
	}
	
	
	
	
	/**
	 * The input theta is the the amount in degrees of the left turn to be made
	 * @param theta
	 */	
	public static void turnLeft(double theta) {
		LightLocalizer.leftMotor.setSpeed(ROTATE_SPEED);
		LightLocalizer.rightMotor.setSpeed(ROTATE_SPEED);
		LightLocalizer.leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
		LightLocalizer.rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
		return;	

	}
	
	
	
	
	/**
	 * The input theta is the the amount in degrees of the right turn to be made
	 * @param theta
	 */		
	public static void turnRight(double theta) {
		LightLocalizer.leftMotor.setSpeed(ROTATE_SPEED);
		LightLocalizer.rightMotor.setSpeed(ROTATE_SPEED);
		LightLocalizer.leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
		LightLocalizer.rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		return;		
	}

	
	
	/**
	 * This method allows us to calculate the distance between two pairs of x and y
	 * @param x1
	 * @param x2
	 * @param y1
	 * @param y2
	 * @return distance between 2 points
	 */
	public static double calculateDistance(double x1, double x2, double y1, double y2) {
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}
	

	
	/**
	 * This method allows us to convert distance we want to travel to the number of rotation the wheels
	 * @param radius of the wheel
	 * @param distance
	 * @return number of rotations
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
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	
	
	
	
	
}
