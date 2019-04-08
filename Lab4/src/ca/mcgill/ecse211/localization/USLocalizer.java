 /** This class localize the car using the light sensor
  * @author Chang 
  * @author Scordos
  */


package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab4.*;

public class USLocalizer {

	/**
	 * constants for our car and for the noise margin
	 */
	public static int ROTATION_SPEED = 100;
	private static final double NOISE_MARGIN_D = 35.00;
	private static final double NOISE_MARGIN_K = 2;
	
	/**
	 * will be used to correct the angle
	 */
	private double deltaTheta;
	private Odometer odometer;
	
	/**
	 * list of data will be used to determine the falling&rising edge
	 */
	private float[] usData;
	private SampleProvider usDistance;
	
	/**
	 * motor
	 */
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	/**
	 * the boolean which tells us to localize with a rising or falling
	 */
	private boolean isRisingEdge;



	/**
	 * Constructor to initialize variables
	 * 
	 * @param Odometer
	 * @param EV3LargeRegulatedMotor for left motor
	 * @param EV3LargeRegulatedMotor for right motor
	 * @param boolean localize with/without rising
	 * @param SampleProvider for detection of distance
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			boolean isRising, SampleProvider usDistance) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.isRisingEdge = isRising;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}

	
	/**
	 * A method to determine which localization method to write
	 * 
	 */
	public void localize() {
		if (isRisingEdge) {
			localizeRisingEdge();
		} else {
			localizeFallingEdge();
		}
	}

	
	/**
	 * A method to localize position using the rising edge
	 * 
	 */
	public void localizeRisingEdge() {

		double angleAlpha, angleBeta, turningAngle;

		// Rotate to the wall
		while (getWallDistace() > NOISE_MARGIN_D) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate until it sees the open space
		while (getWallDistace() < NOISE_MARGIN_D + NOISE_MARGIN_K) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.buzz();
		// record angle
		angleAlpha = odometer.getXYT()[2];

		// rotate the other way all the way until it sees the wall
		while (getWallDistace() > NOISE_MARGIN_D) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate until it sees open space
		while (getWallDistace() < NOISE_MARGIN_D + NOISE_MARGIN_K) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.buzz();
		angleBeta = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleAlpha < angleBeta) {
			deltaTheta = 45 - (angleAlpha + angleBeta) / 2 + 180;
		} else if (angleAlpha > angleBeta) {
			deltaTheta = 225 - (angleAlpha + angleBeta) / 2 + 180;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 using turning angle and we account for small
		// error
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle), false);

		// set theta to zero
		odometer.setXYT(0.0,0.0,0.0);
	}

	
	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void localizeFallingEdge() {

		double angleAlpha, angleBeta, turningAngle;

		// rotate counter-clock-wisely
		while (getWallDistace() < NOISE_MARGIN_D + NOISE_MARGIN_K) {
			leftMotor.backward();
			rightMotor.forward();
		}
		
		// rotate to the first falling angle
		while (getWallDistace() > NOISE_MARGIN_D) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.buzz();
		
		// record the first falling angle	
		angleAlpha = odometer.getXYT()[2];

		// rotate clock-wisely
		while (getWallDistace() < NOISE_MARGIN_D + NOISE_MARGIN_K) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second falling angle
		while (getWallDistace() > NOISE_MARGIN_D) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.buzz();
		
		//record the second falling angle
		angleBeta = odometer.getXYT()[2];

		//stop rotating
		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		// this calculation is from the tutorial
		if (angleAlpha < angleBeta) {
			deltaTheta = 45 - (angleAlpha + angleBeta) / 2;
		} else if (angleAlpha > angleBeta) {
			deltaTheta = 225 - (angleAlpha + angleBeta) / 2;
		}
		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 and we account for small error
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, turningAngle), false);

		// reset odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}

	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return
	 */
	private int getWallDistace() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
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
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
