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
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	//the two arm motors for capturing the block
//	private static final EV3LargeRegulatedMotor armMotor1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
//	private static final EV3LargeRegulatedMotor armMotor2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	//sensor ports


	//robot dimension constants
	public static final double ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR = 10.6;
	public static final double WHEEL_RADIUS = 2.09;
	public static final double TRACK = 15.15; 
	
	public static void main(String[] args)  {
		
		//Setup ultrasonic sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
		UltrasonicPoller frontPoller = new UltrasonicPoller("S4");
		UltrasonicPoller sidePoller = new UltrasonicPoller("S1");
		ColorSensorPoller blockPoller = new ColorSensorPoller("S3");
		
		// start the block detector thread, which will be constantly checking with the light sensor
		//to see if there is a block.
//		BlockDetector blockDetector = new BlockDetector(colorValue, colorData);
//		blockDetector.start();	
//		
		// setup the odometer
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		//setup the wall avoider
		WallAvoider avoider = new WallAvoider(odo, frontPoller, sidePoller);
		
		//set up the display and navigator
		LCDInfo lcd = new LCDInfo(odo,frontPoller,sidePoller, blockPoller);
		Navigation navi = new Navigation(odo, avoider, frontPoller, WHEEL_RADIUS, TRACK);
		
		//set up the light localization
		LightLocalizer lsl = new LightLocalizer(odo, blockPoller, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR);
		USLocalizer usl = new USLocalizer(odo,navi, frontPoller, USLocalizer.LocalizationType.FULL_CIRCLE);
		
		/*
		 * We wait for a press. If it is a left button, we're just doing the detection
		 * otherwise we do the block stuff.
		 */
		int buttonPressed = Button.waitForAnyPress();
		if(buttonPressed == Button.ID_LEFT){
			
		}else{ 
			// perform the ultrasonic localization
			//Rising edge was found to be the best in this case! So we use that one.
			//disable the side sensor for localization so that it doens't interfere
			sidePoller.disableSensor();
			usl.doLocalization();
			lsl.doLocalization();
			sidePoller.enableSensor();
			//double[] pos = {0, 0,0};
			//boolean[] up = {true,true,true};
			//odo.setPosition(pos,up);
			navi.travelToAndAvoid(60.8, 60.8);
			//leftMotor.setSpeed(10);
			//leftMotor.forward();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}
