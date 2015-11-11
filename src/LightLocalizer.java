import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
/**
 * This class implements a light sensor localization.
 * @author Yan Ren
 * @version 1.0
 *
 */
public class LightLocalizer {
	//constants
	private double del;         //distance from the color sensor to the centre of robot.
	private static final long CORRECTION_PERIOD = 20;
	private double brightnessThreshold = 0.40;
	//variables
	long correctionStart, correctionEnd;
	private Navigation navigation;
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;	
	private double brightness;		
	private int numberOfGridLines = 0;
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
	
	/**
	 * Constructor for the light localizer
	 * @param odo - The odometer being used by the robot
	 * @param colorSensor - The color sensor that the robot is using
	 * @param colorData - The float array containing the color data of the sensor
	 * @param navigation - The navigation class being used by the robot
	 */
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData, Navigation navigation, double del) {
		this.navigation = navigation;
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.del = del;
	}
	
	/**
	 * Uses the color sensor to perform a "localization", where it figures out its initial starting angle.
	 * After localization, robot will travel to (0,0) point and rotate to X positive
	 */
	public void doLocalization() {
		
		odo.setPosition(new double[] {0.0, 0.0, 0.0} , new boolean[] {true, true, true} );
		navigation.rotateForLightLocalization();
		double angle = 0;
		
		while (navigation.isRotating() == true){
			correctionStart = System.currentTimeMillis();
			colorSensor.fetchSample(colorData, 0);
			brightness = colorData[0];
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
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {}
			}
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
				
		//update the odometer position
		odo.setPosition(new double[] {x, y, theta}, new boolean[] {true, true, true});
		
		travelToOrigin();
	}
	
	/**
	 * After localization, this method help robot travel to (0,0) point and rotate to X positive
	 */
	private void travelToOrigin(){
		 navigation.travelTo(0.0, 0.0);
		 navigation.turnTo(0.0, true);
	}
}
