/** This class localize the car using the light sensor
 * 
 * @author Chang
 * @author Scordos
 */
package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.lab4.*;



public class LightLocalizer {

	/**
	 * constants for our car
	 */
	private static final int ROTATION_SPEED = 100;
	private static final double SENSOR_DISTANCE = 9.0;
	
	/**
	 * constants will be used in calculation
	 */
	private static final double ROUND_ANGLE = 360;
	private static final double RIGHT_ANGLE = 90;
	
	/**
	 * threshold for light sensor detection
	 */
	private static final double COLOR_THRESHOLD_A = 0.20;
	private static final double COLOR_THRESHOLD_B = 0.38;
	
	/**
	 * odometer for data correction
	 */
	public static Odometer odometer;
	
	/**
	 * car motors for navigate
	 */
	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	

	/**
	 * Instantiate the EV3 Color Sensor
	 */
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
	private float sample;
	private SensorMode idColour;
	double[] thetaOnLine;

	/**
	 * Constructor for our light localizer
	 * @param odometer
	 * @param leftMotor
	 * @param rightMotor
	 */
	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

		LightLocalizer.odometer = odometer;
		LightLocalizer.leftMotor = leftMotor;
		LightLocalizer.rightMotor = rightMotor;

		idColour = lightSensor.getRedMode(); // set the sensor light to red
		thetaOnLine = new double[4];
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 */
	public void localize() {
		
		//initialize the index of the number of lines being detected
		int index = 0;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		
		
		// ensure that we are in the right postion before rotating
		findPostion();

		
		// Scan all four lines and record our angle
		while (index < 4) {	
			
			//rotating clock-wisely
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.forward();
			rightMotor.backward();
			
			//fetch sample continuously
			sample = fetchSample();
			if (sample < COLOR_THRESHOLD_A) {
				//get the theta for positive x axis, negative y axis, negative x axis, and positive y axis
				thetaOnLine[index] = odometer.getXYT()[2];
				Sound.beep();
				index++;
			}
		}
		
		//stop when we detect the four line which is the positive y axis
		leftMotor.stop(true);
		rightMotor.stop();

		//now calculate the current position by trigonometry
		double thetaX, thetaY, deltaX, deltaY, deltaTheta;
		thetaX=thetaOnLine[1] - thetaOnLine[3];
		thetaY=thetaOnLine[0] + ROUND_ANGLE - thetaOnLine[2];
		deltaX = -1 * SENSOR_DISTANCE * Math.cos(Math.toRadians(thetaX / 2));
		deltaY = -1 * SENSOR_DISTANCE * Math.cos(Math.toRadians(thetaY / 2));
		deltaTheta=odometer.getTheta();
		
		
		// set the calculated position
		odometer.setXYT(deltaX,deltaY,deltaTheta);
		
		//go to the origin and turn to 0 degree(heading in positive y)
		Navigation.travelTo(0,0);
		Navigation.turnTo(0);

	}

	
	/**
	 * This method moves the robot's sensor a bit in front of the origin
	 * so that the car can start rotating
	 */
	public void findPostion() {

		//turn to the angle which face the origin
		LightLocalizer.leftMotor.setSpeed(ROTATION_SPEED);
		LightLocalizer.rightMotor.setSpeed(ROTATION_SPEED);
		LightLocalizer.leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK,RIGHT_ANGLE/2), true);
		LightLocalizer.rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, RIGHT_ANGLE/2), false);


		// get sample
		sample = fetchSample();

		// move forward past the origin until light sensor sees the line
		while (sample > COLOR_THRESHOLD_B) {
			sample = fetchSample();
			leftMotor.forward();
			rightMotor.forward();
		}
		
		//stop the motor
		leftMotor.stop();
		rightMotor.stop();

		// Move forwards so our origin is close to origin
		LightLocalizer.leftMotor.setSpeed(ROTATION_SPEED);
		LightLocalizer.rightMotor.setSpeed(ROTATION_SPEED);
		leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 12), true);
		rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 12), false);


	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
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
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	
	
	
	/**
	 * This method gets the color value of the light sensor
	 * @return a float represents the color value
	 */
	private float fetchSample() {
		float[] colorValue = new float[idColour.sampleSize()];
		idColour.fetchSample(colorValue, 0);
		return colorValue[0];
	}

}
