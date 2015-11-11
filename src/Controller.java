/*
 * Beta demo.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 5 - Finding Objects
 * Group 53
 * This class sets up the classes for Finding the objects, and calls them. It also initializes the sensors
 * and the motors.
 */
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Controller {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	//the two arm motors for capturing the block
//	private static final EV3LargeRegulatedMotor armMotor1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
//	private static final EV3LargeRegulatedMotor armMotor2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	//sensor ports
	private static final Port usPort = LocalEV3.get().getPort("S1");		
	private static final Port colorPort = LocalEV3.get().getPort("S2");		
	//robot dimension constants
	public static final double ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR = 10.6;
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 15.2; 
	
	public static void main(String[] args)  {
		
		//Setup ultrasonic sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
		@SuppressWarnings("resource")							    	// Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance");			// colorValue provides samples from this instance
		float[] usData = new float[usValue.sampleSize()];				// colorData is the buffer in which data are returned
		
		//Setup color sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
  		SensorModes colorSensor = new EV3ColorSensor(colorPort);
	    SampleProvider colorValue = colorSensor.getMode("Red");  		// colorValue provides samples from this instance
		float[] colorData = new float[colorValue.sampleSize()];			// colorData is the buffer in which data are returned
	
		// start the block detector thread, which will be constantly checking with the light sensor
		//to see if there is a block.
//		BlockDetector blockDetector = new BlockDetector(colorValue, colorData);
//		blockDetector.start();	
//		
		// setup the odometer
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		//this commented out code is used if we are testing robot and do not want to localize
		/*boolean[] update= new boolean[3];
		update[0] = false; update[1] = false; update[2] = true;
		double[] pos = new double[3];
		pos[0] = 0; pos[1] = 0; pos[2] = 90;
		odo.setPosition(pos,update);*/
		
		//set up the display and navigator
		LCDInfo lcd = new LCDInfo(odo, usValue, usData);
		Navigation navi = new Navigation(odo, WHEEL_RADIUS, TRACK);
		
		//set up the light localization
		LightLocalizer lsl = new LightLocalizer(odo, colorValue, colorData, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR);

		/*
		 * We wait for a press. If it is a left button, we're just doing the detection
		 * otherwise we do the block stuff.
		 */
		int buttonPressed = Button.waitForAnyPress();
		if(buttonPressed == Button.ID_LEFT){
			
		}else{ 
			// perform the ultrasonic localization
			//Rising edge was found to be the best in this case! So we use that one.
			USLocalizer usl = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationType.FALLING_EDGE);
			
			usl.doLocalization();
			lsl.doLocalization();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}
