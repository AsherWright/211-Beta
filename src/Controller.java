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
	private static final EV3LargeRegulatedMotor verticalArmMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor horizontalArmMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	//sensor ports
	

	//robot dimension constants
	public static final double ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR = 10.6;
	public static final double WHEEL_RADIUS = 2.09;
	public static final double TRACK = 15.15; 
	
	public static void main(String[] args)  {
		double zoneX = 4*30.4;
		double zoneY = 6*30.4;
		//Setup ultrasonic sensor
		// 1. Create a port object attached to a physical port (done above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data
//		UltrasonicPoller frontPoller = new UltrasonicPoller("S4");
		UltrasonicPoller frontPoller = new UltrasonicPoller("S4");
		UltrasonicPoller sidePoller = new UltrasonicPoller("S1");
		ColorSensorPoller blockPoller = new ColorSensorPoller("S2");
		ColorSensorPoller groundPoller = new ColorSensorPoller("S3");
		groundPoller.setMode(1);
		// start the block detector thread, which will be constantly checking with the light sensor
		//to see if there is a block.

//		
		// setup the odometer
		Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		//setup the wall avoider
		WallAvoider avoider = new WallAvoider(odo, frontPoller, sidePoller);
//		WallAvoider avoider = new WallAvoider(odo, frontPoller, null);
		//set up the display and navigator
		LCDInfo lcd = new LCDInfo(odo,frontPoller,sidePoller, blockPoller);
//		LCDInfo lcd = new LCDInfo(odo,frontPoller,null, null);
		Navigation navi = new Navigation(odo, avoider, frontPoller, WHEEL_RADIUS, TRACK);
		BlockDetector blockDetector = new BlockDetector(blockPoller, navi, odo, frontPoller, verticalArmMotor, horizontalArmMotor);
		blockDetector.start();	
		SearchingField searcher = new SearchingField(leftMotor, rightMotor, sidePoller, frontPoller, navi, odo, blockDetector, zoneX, zoneY);

		//set up the light localization
		LightLocalizer lsl = new LightLocalizer(odo, groundPoller, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR);
//		LightLocalizer lsl = new LightLocalizer(odo, null, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR);
		USLocalizer usl = new USLocalizer(odo,navi, frontPoller, USLocalizer.LocalizationType.FULL_CIRCLE);
		
		/*
		 * We wait for a press. If it is a left button, we're just doing the detection
		 * otherwise we do the block stuff.
		 */
		int buttonPressed = Button.waitForAnyPress();
		if(buttonPressed == Button.ID_LEFT){
			
		}else{ 
			//disable the side sensor for localization so that it doens't interfere
			sidePoller.disableSensor();
			navi.setCmError(0.4);
			navi.setDegreeError(4.0);
			usl.doLocalization();
			
			navi.setCmError(0.2);
			navi.setDegreeError(2.0);
			navi.setSlowSpeed(120);
			lsl.doLocalization();
			sidePoller.enableSensor();
			navi.setSlowSpeed(90);
			navi.travelToAndAvoid(zoneX - 10, zoneY-2*30.4);
			navi.travelTo(zoneX-10, zoneY);
			navi.turnTo(250, true);
			odo.setTheta(270);
			//the block searcher should go here
			searcher.run();
			navi.travelToAndAvoid(0, 0);
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}
