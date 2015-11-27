import pollers.ColorSensorPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
/**
 * This class implements a light sensor localization.
 * @author Yan Ren
 * @version 1.0
 *
 */
import wifi.StartCorner;

public class LightLocalizer {
	//constants
	private double del;         //distance from the color sensor to the centre of robot.
	/**
	 * This constant controls the period(millisecond) in which lightLocalizer requires data from ColorSensorPoller
	 */
	private static final long DATA_PERIOD = 20;
	private double brightnessThreshold = 0.40;
	//variables
	private StartCorner startingCorner = StartCorner.NULL;
	long correctionStart, correctionEnd;
	private Navigation navigation;
	private Odometer odo;
	private double brightness;		
	private boolean isBlackLine = false;
	/**
	 * Holds intermediate variables for calculating x, y, theta 
	 */
	private double thetaYNegative;
	private double thetaYPositive;
	private double thetaXNegative;
	private double thetaXPositive;
	private double deltaThetaY;   
	/**
	 * Holds current x, y, theta after localization
	 */
	private double deltaThetaX;   
	private double x, y, theta;   
	private ColorSensorPoller groundPoller;
	
	/**
	 * Constructor for the light localizer
	 * @param odo - The odometer being used by the robot
	 * @param colorSensor - The color sensor that the robot is using
	 * @param colorData - The float array containing the color data of the sensor
	 * @param navigation - The navigation class being used by the robot
	 */
	public LightLocalizer(Odometer odo, ColorSensorPoller groundPoller, Navigation navigation, double del) {
		this.navigation = navigation;
		this.odo = odo;
		this.groundPoller = groundPoller;
		this.del = del;
	}
	public LightLocalizer(Odometer odo, ColorSensorPoller groundPoller, Navigation navigation, double del, StartCorner startingCorner) {
		this.navigation = navigation;
		this.odo = odo;
		this.groundPoller = groundPoller;
		this.del = del;
		this.startingCorner = startingCorner;
	}
	/**
	 * Uses the color sensor to perform a "localization", where it figures out its initial starting angle.
	 * After localization, robot will travel to (0,0) point and rotate to X positive
	 * Inside doLocalization() exits a time control, so that function only requires data once from color sensor every CORRECTION_PERIOD.
	 */
	public void doLocalization() {
		
		//setting the color sensor properties.
		groundPoller.setPollRate(20);
		int numberOfGridLines = 0;
		double angle = 0;
		odo.setPosition(new double[] {0.0, 0.0, 0.0} , new boolean[] {true, true, true} );
		
		//start localization
		navigation.rotateForLightLocalization();		
		while (navigation.isRotating() == true){
			correctionStart = System.currentTimeMillis();
			brightness = groundPoller.getBrightness();
			if (brightness < brightnessThreshold)
			{
				//get angle from odometer
				angle = odo.getAng();
				//light sensor is crossing the grid line
				isBlackLine = true;
				Sound.beep();
				numberOfGridLines = numberOfGridLines + 1;
			}
			else 
			{
				isBlackLine = false;
			}
	    
			if (isBlackLine == true){
				if (numberOfGridLines == 1){ 
					thetaYNegative = angle;								
				}
				else if (numberOfGridLines == 2){
					thetaXPositive = angle;				
				}		
				else if (numberOfGridLines == 3){ 
					thetaYPositive = angle;
				}
				else if (numberOfGridLines == 4){
					thetaXNegative = angle;
					
				}		
			 }
			
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < DATA_PERIOD) {
				try {
					Thread.sleep(DATA_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {}
			}
		}
		
		// check if the robot intersects with grid lines four times
		if(numberOfGridLines != 4){
			return;
		}
		
		// do trig to compute x,y and theta
		deltaThetaY = thetaYPositive - thetaYNegative;
		deltaThetaX = thetaXNegative - thetaXPositive;
		x = -1*del*Math.cos(Math.toRadians(deltaThetaY/2));
		y = -1*del*Math.cos(Math.toRadians(deltaThetaX/2));
		
		theta = 180.0 - thetaYNegative;
		if (theta >= deltaThetaY/2){
			theta = theta - deltaThetaY/2;
		}else{
			theta = 360.0 - (deltaThetaY/2 - theta);
		}
		
		//update the current position to odometer and send it to (0,0) and pointing to x-positive
		odo.setPosition(new double[] {x, y, theta}, new boolean[] {true, true, true});	
//		Button.waitForAnyPress();
//		travelToOrigin();
//		navigation.stopMotor();
		
		//Calculate the actual coordinates and direction according to starting corner
		if(startingCorner.getId() == 2){
			x = odo.getX() + startingCorner.getX();
			y = odo.getY() + startingCorner.getY();
			theta = odo.getAng() + 90.0;
		}else if(startingCorner.getId() == 3){
			x = odo.getX() + startingCorner.getX();
			y = odo.getY() + startingCorner.getY();
			theta = odo.getAng() + 180.0;
		}else if(startingCorner.getId() == 4){
			x = odo.getX() + startingCorner.getX();
			y = odo.getY() + startingCorner.getY();
			theta = odo.getAng() + 270.0;
		}else{
			x = odo.getX();
			y = odo.getY();
			theta = odo.getAng(); 
		}
		odo.setPosition(new double[] {x, y, theta}, new boolean[] {true, true, true});					
	}
	
	/**
	 * After localization, this method help robot travel to (0,0) point and rotate to X positive
	 */
	private void travelToOrigin(){
		 navigation.travelTo(0.0, 0.0);
		 navigation.turnTo(0.0, true);
	}
	/**
	 * 
	 * @param lsl_point_x 
	 * @param lsl_point_y
	 */
	public void reLocalization(double lsl_point_x, double lsl_point_y){
		
		double[] startPosition = odo.getPosition();
		int numberOfGridLines = 0;
		double angle = 0;
		// setting the color sensor properties.
		groundPoller.setPollRate(20);
		
		// start localizaiton
		navigation.turnTo(45.0, true);
		odo.setPosition(new double[] {0.0, 0.0, 0.0} , new boolean[] {true, true, true} );
		navigation.rotateForLightLocalization();
			
		while (navigation.isRotating() == true){
			correctionStart = System.currentTimeMillis();
			brightness = groundPoller.getBrightness();
		if (brightness < brightnessThreshold)
		{
			//get angle from odometer
			angle = odo.getAng();
			//light sensor is crossing the grid line
			isBlackLine = true;
			Sound.beep();
			numberOfGridLines = numberOfGridLines + 1;
		}
		else 
		{
			isBlackLine = false;
		}
    
		if (isBlackLine == true){
			if (numberOfGridLines == 1){ 
				thetaYNegative = angle;								
			}
			else if (numberOfGridLines == 2){
				thetaXPositive = angle;				
			}		
			else if (numberOfGridLines == 3){ 
				thetaYPositive = angle;
			}
			else if (numberOfGridLines == 4){
				thetaXNegative = angle;
				
			}		
		 }
		
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < DATA_PERIOD) {
				try {Thread.sleep(DATA_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {}
			}
		}
		// check if the robot intersects with grid lines four times
		if(numberOfGridLines != 4){
			odo.setPosition(startPosition, new boolean[] {true, true, true});
			return;
		}
	
		// do trig to compute x,y and theta
		deltaThetaY = thetaYPositive - thetaYNegative;
		deltaThetaX = thetaXNegative - thetaXPositive;
		x = -1*del*Math.cos(Math.toRadians(deltaThetaY/2));
		y = -1*del*Math.cos(Math.toRadians(deltaThetaX/2));
	
		theta = 180.0 - thetaYNegative;
		if (theta >= deltaThetaY/2){
			theta = theta - deltaThetaY/2;
		}else{
			theta = 360.0 - (deltaThetaY/2 - theta);
		}
	
		//update the current position to odometer. Based on these value, the robot will go to (0,0) and pointing to x-positive
//		odo.setPosition(new double[] {x, y, theta}, new boolean[] {true, true, true});		
//		travelToOrigin();
//		navigation.stopMotor();
	
		//Calculate the actual coordinates and direction according to starting corner		
		odo.setPosition(new double[] {x + lsl_point_x, y + lsl_point_y, theta}, new boolean[] {true, true, true});					
		}
}
