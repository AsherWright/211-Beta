
import pollers.ColorSensorPoller;
import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;

/* 
 * OdometryCorrection.java
 */
public class OdometerCorrection extends Thread {
	
	//variables
	private static final long CORRECTION_PERIOD = 10;
	//distance from the sensor to the centre of rotation
	private static final double SENSORDIST = 4.5;
	//the minimum value of RGB that could be called white.
	private static final double MINIMUMWHITEVALUE = 0.20;
	private double brightnessThreshold = 0.40;
	private Odometer odometer;
	private ColorSensorPoller linePoller;
	//first and current brightnesses.
	private double firstBrightnessLevel;
	private double currBrightnessLevel;
	//the percent difference in our reading to consider it a different color (used for reading black)
	private double significantPercentThreshold = 20;
	//array to store the measured RGB values
	private float[] RGBValues = new float[3];
	private boolean reachedBlackLine = false;
	//variables to see if it is the first X or Y correction. It always starts in same first square (-15,-15)-->(15,15)
	private boolean isFirstXCorrection = true;
	private boolean isFirstYCorrection = true;
	
	// constructor
	public OdometerCorrection(Odometer odometer, ColorSensorPoller linePoller) {
		this.odometer = odometer;
		this.linePoller = linePoller;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		Long lastCorrection = System.currentTimeMillis();
		//colorSensor.setFloodlight(lejos.robotics.Color.WHITE); //we set our light to White (we use white over R)
		while (true) {
			correctionStart = System.currentTimeMillis();
		
			//we define the brightness as the average of the magnitudes of R,G,B (really "Whiteness")
			currBrightnessLevel = linePoller.getBrightness();
			
			/*
			 * If it is our first brightness level, we just set it to our measured
			 * Else, it is not our FIRST measurement, so we check to see if we hit a black line. 
			 */
			if (currBrightnessLevel < brightnessThreshold && Math.abs(lastCorrection-System.currentTimeMillis()) > 2000){	
				lastCorrection = System.currentTimeMillis();
				//we only want to correct it every so and so seconds...
				//if we've reached a black line, correct the position of the robot.
				correctOdometerPosition();
//				Sound.beep();
			}
			
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
	/*
	 * Corrects the odometer's position (is triggered on hitting a black line).
	 */
	private void correctOdometerPosition(){

		//variables
		double currX = odometer.getX();
		double currY = odometer.getY();
		double currT = odometer.getAng();
		double offset = 10.6;
		//find out what the position of our black line hit was (where the light sensor is).
		double blackLineX = currX - offset*Math.cos(currT*2*Math.PI/360.0);
		double blackLineY = currY - offset*Math.sin(currT*2*Math.PI/360.0);

		
		//now figure out where these x and y could possibly point to.
		int xTile = (int) (blackLineX / 30.4);
		int yTile = (int) (blackLineY / 30.4);
		double linethreshold = 10;
		double xOff = Math.abs(blackLineX - xTile*30.4);
		double yOff = Math.abs(blackLineY - yTile*30.4);
		//create an array for what fields to update for our robot and set values
		boolean[] update = new boolean[3];		//create an array for the position of our robot and set the values
		double[] position = new double[3];
		update[0] = false;
		update[1] = false;
		update[2] = false;
		if(xOff < yOff && xOff < linethreshold){
			position[0] = xTile*30.4 +offset*Math.cos(currT*2*Math.PI/360.0);
			update[0] = true;
//			Sound.beep();
		}else if (yOff < xOff && yOff < linethreshold){
			position[1] = yTile*30.4 +offset*Math.sin(currT*2*Math.PI/360.0);
			update[1] = true;
//			Sound.beep();
		}

		//now use those two arrays to set the position of the odometer (it is now corrected).
		odometer.setPosition(position,update);
		
	}

}