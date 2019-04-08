/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double TILE_SIZE=30.48;
  private static final double TURN_ERROR=1.70;//the sensor's position changes when it perform's a 90 degree turn because it is placing in front of the wheel
  private Odometer odometer;
  
  private static final Port portColor = LocalEV3.get().getPort("S1");
  SensorModes myColor= new EV3ColorSensor(portColor);
  SampleProvider myColorStatus=myColor.getMode("RGB");
  float[] sampleColor=new float[myColorStatus.sampleSize()];
  
  private int numberOfLines = 0;
  private int lastNumberOfLines = 0;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;


    while (true) {
      correctionStart = System.currentTimeMillis();
      myColorStatus.fetchSample(sampleColor, 0);
      
      // TODO Trigger correction (When do I have information to correct?)
      
      //each time it detect a black line, adjust the position and beep
      if (sampleColor[0]<0.1) {
    	  	Sound.beep();
		numberOfLines++;
      }

      
      // TODO Calculate new (accurate) robot position
      // TODO Update odometer with new calculated (and more accurate) vales
      
      if (numberOfLines>lastNumberOfLines && numberOfLines>=1) {
    	  	double positionCurrent[]=odometer.getXYT();
    	  	//adjust y
    	  	if (numberOfLines==1||numberOfLines==2||numberOfLines==3) { 
    	  		odometer.setXYT(positionCurrent[0],(numberOfLines-1)*TILE_SIZE,positionCurrent[2]);
    	  	} 
    	  	//adjust x
    	  	if (numberOfLines==4||numberOfLines==5||numberOfLines==6) {
    	  		odometer.setXYT((numberOfLines-4)*TILE_SIZE,positionCurrent[1],positionCurrent[2]);
    	  	}
    	  	//adjust y
    	  	if (numberOfLines==7||numberOfLines==8||numberOfLines==9) {
    	  		odometer.setXYT(positionCurrent[0],(9-numberOfLines)*TILE_SIZE,positionCurrent[2]);
    	  	}  	  
    	  	//adjust x
    	  	if (numberOfLines==10||numberOfLines==11) {
    	  		odometer.setXYT((12-numberOfLines)*TILE_SIZE,positionCurrent[1],positionCurrent[2]);
    	  	}  	
    	  	//adjust x by set x to 0 & adjust y by adding the turn error
    	  	if (numberOfLines==12) {
    	  		odometer.setXYT((12-numberOfLines)*TILE_SIZE,positionCurrent[1]+TURN_ERROR,positionCurrent[2]);
    	  	}
      }
      
      //update total number of lines
      lastNumberOfLines=numberOfLines;
     
      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
