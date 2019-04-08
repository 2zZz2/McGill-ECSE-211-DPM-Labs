/** This class is the main entrance for our software
 * 
 * @author Chang
 * @author Scordos
 */
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.localization.*;



public class Lab4 {

	/**
	 * connection to our hardware
	 */
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static boolean isRisingEdge = true;

	/**
	 * properties of our car
	 */
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.4;

	
	/**
	 * This main method is the entrance point of our software
	 */
	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// setup our odometer
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); 

		
		//set up our sensor
		@SuppressWarnings("resource") 
		// usSensor is the instance
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);
		// usDistance provides samples from this instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");

		
		do {
			// clear the display
			lcd.clear();

			// ask the user whether to choose the falling edge or the rising edge
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising |Falling ", 0, 2);
			lcd.drawString(" Edge  |  Edge  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);


		if (buttonChoice == Button.ID_LEFT) {
			isRisingEdge = true;
		} else {
			isRisingEdge = false;
		}

		// Start odometer & odometer display threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Create ultrasonic and light localizer objects.
		USLocalizer USLocalizer = new USLocalizer(odometer, leftMotor, rightMotor, isRisingEdge, usDistance);
		LightLocalizer lightLocatizer = new LightLocalizer(odometer, leftMotor, rightMotor);

		
		
		// perform the ultrasonic localization
		USLocalizer.localize();
		if(Button.waitForAnyPress() == Button.ID_DOWN) {
			System.exit(0);
		}
		
		// perform the light localization
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		lightLocatizer.localize();
		
		//exit the program
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}