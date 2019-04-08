package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Lab3 {
	
	/**
	 * constants that represent properties of our car
	 */

	private static final double WHEEL_RAD = 2.1;
	private static final double TRACK = 13.4; 
	
	/**
	 * connections to our car's hardware
	 */
	
	public static final EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));	
	public static final TextLCD LCD = LocalEV3.get().getTextLCD();
	
	
	/**
	 * setup for the threads later
	 */
	public static Odometer odometer;
	public static Display odometryDisplay;

	
	/**
	 * entrance for our software
	 */
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {
		
		int buttonChoice;
		
		do {
		      // clear the display
		      LCD.clear();	      
		      // ask the user whether the motors should drive with / without blocks
		      LCD.drawString("< Left | Right >", 0, 0);
		      LCD.drawString("       |        ", 0, 1);
		      LCD.drawString(" No    | With   ", 0, 2);
		      LCD.drawString(" Blocks| Blocks ", 0, 3);
		      LCD.drawString("       |        ", 0, 4);
		      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);	
		
		
		//with out the blocks
		if (buttonChoice == Button.ID_LEFT ) {
			//building the threads
			
			odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);
			odometryDisplay = new Display(LCD);
			Navigation navigation = new Navigation();
			
			Thread odoThread = new Thread(odometer);
			Thread odoDisplayThread = new Thread(odometryDisplay);	
			Thread naviagtionThread = new Thread(navigation);
			
			odoThread.start();
			odoDisplayThread.start();
			naviagtionThread.start();

			while (Button.waitForAnyPress()!=Button.ID_ESCAPE);
		    System.exit(0);
		}
		
		
		//with the blocks
		if (buttonChoice == Button.ID_RIGHT) {
				
			//building the threads
			odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);
			odometryDisplay = new Display(LCD);
			NavigationObstacle navigationObstacle = new NavigationObstacle(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);
				
			Thread navigationNoThread = new Thread(navigationObstacle);
			Thread odoThread = new Thread(odometer);
			Thread odoDisplayThread = new Thread(odometryDisplay);
			navigationNoThread.start();
			odoThread.start();
			odoDisplayThread.start();
		
			while (Button.waitForAnyPress()!=Button.ID_ESCAPE);
			    System.exit(0);
		}
		
	}
}
