
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
		
		firstBrightnessLevel = -1; //start the first brightness at -1 (to know it is first reading)
		//colorSensor.setFloodlight(lejos.robotics.Color.WHITE); //we set our light to White (we use white over R)
		while (true) {
			correctionStart = System.currentTimeMillis();
			
			Long lastCorrection = System.currentTimeMillis();
			
			//we define the brightness as the average of the magnitudes of R,G,B (really "Whiteness")
			currBrightnessLevel = linePoller.getBrightness();
			
			/*
			 * If it is our first brightness level, we just set it to our measured
			 * Else, it is not our FIRST measurement, so we check to see if we hit a black line. 
			 */
			if (currBrightnessLevel< brightnessThreshold && Math.abs(lastCorrection-System.currentTimeMillis()) > 2000){	
				lastCorrection = System.currentTimeMillis();
				//we only want to correct it every so and so seconds...
				//if we've reached a black line, correct the position of the robot.
				correctOdometerPosition();
				
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
		double correctedX = 0;
		double correctedY = 0;
		double offset = 10.6;
		//find out what the position of our black line hit was (where the light sensor is).
		double blackLineX = currX - offset*Math.cos(currT);
		double blackLineY = currY - offset*Math.sin(currT);
		
		//now figure out where these x and y could possibly point to.
		int xTile = (int) (blackLineX / 30.4);
		int yTile = (int) (blackLineY / 30.4);
		double linethreshold = 5;
		double xOff = Math.abs(blackLineX - xTile);
		double yOff = Math.abs(blackLineY - yTile);
		//create an array for what fields to update for our robot and set values
		boolean[] update = new boolean[3];		//create an array for the position of our robot and set the values
		double[] position = new double[3];
		update[0] = false;
		update[1] = false;
		update[2] = false;
		if(xOff < yOff && xOff < linethreshold){
			position[0] = xTile*30.4;
			update[0] = true;
			Sound.beep();
		}else if (yOff < xOff && yOff < linethreshold){
			position[1] = yTile*30.4;
			update[1] = true;
			Sound.beep();
		}

//		position[0] = correctedX;
//		position[1] = correctedY;
//		position[2] = 0;

		//now use those two arrays to set the position of the odometer (it is now corrected).
		odometer.setPosition(position,update);
		
	}
	/*
	 * Finds the "corrected" X value, assuming (0,0) is in the middle of the first square,
	 * and blocks are 30x30. 
	 */
	private double findCorrectedX(double x){
		double result = x;
		/*
		 * if it's the first correction, we know that X should be 15. otherwise find the nearest line value
		 * to where we are. If none are close enough, keep it the same (false reading).
		 */
		if(isFirstXCorrection){
			result = 15;
			isFirstXCorrection = false;
		}else{
			for(int i = 0; i < 4; i++){
				if(Math.abs(x - (15-SENSORDIST + 30*i)) < 12){
					result = 15 + 30*i;
					break;
				}
			}	
		}
		return result;
	}
	/*
	 * Finds the "corrected" Y value, assuming (0,0) is in the middle of the first square,
	 * and blocks are 30x30.
	 */
	private double findCorrectedY(double y){
		double result = y;
		/*
		 * if it's the first correction, we know that Y should be -15. otherwise find the nearest line value
		 * to where we are. If none are close enough, keep it the same (false reading).
		 */
		if(isFirstYCorrection){
			result = -15;
			isFirstYCorrection = false;
		}else{
			for(int i = 0; i < 4; i++){
				if(Math.abs(y + (15-SENSORDIST + 30*i)) < 12){
					result = (-1)*(15 + 30*i);
					break;
				}
			}
		}
		return result;
	}
	/*
	 * Finds the nearest right angle (0,90,180,270,etc) to the robots direction
	 * to determine if we are moving in the X or the Y. We call X 0 and Y 90.
	 */
	private int findRightAngleRobotDirection(int t){
		int result = 0;
		int allowedError = 5;
		/*
		 * checks to see if we are in the allowed range for any of the right angles.
		 */
		if (Math.abs(t) < allowedError || Math.abs(t-360) < allowedError || Math.abs(t+360) < allowedError || Math.abs(t-180) < allowedError || Math.abs(t+180) < allowedError){
			result = 0;
		}else if (Math.abs(t-90) < allowedError || Math.abs(t+90) < allowedError || Math.abs(t-270) < allowedError || Math.abs(t+270) < allowedError){
			result = 90;
		}
		
		return result;
	}
	
	//accessors used for displaying text on LCD.
	public boolean isReadingBlack(){
		return reachedBlackLine;
	}
	public double getR(){
		return RGBValues[0];
	}
	public double getG(){
		return RGBValues[1];
	}
	public double getB(){
		return RGBValues[2];
	}
	public double getBrightness(){
		return currBrightnessLevel;
	}
}