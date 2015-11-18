/*
 * Beta demo.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 5 - Finding Objects
 * Group 53
 * This class sets up the classes for Finding the objects, and calls them. It also initializes the sensors
 * and the motors.
 */
import java.io.IOException;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import wifi.Transmission;
import wifi.WifiConnection;

public class Controller {
	//variables for WiFi module 
	// *** INSTRUCTIONS ***
	// There are two variables to set manually on the EV3 client:
	// 1. SERVER_IP: the IP address of the computer running the server application
	private static final String SERVER_IP = "192.168.10.108";
	private static final int TEAM_NUMBER = 14;	
	private static TextLCD LCD = LocalEV3.get().getTextLCD();
	//

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	//the two arm motors for capturing the block
	private static final EV3LargeRegulatedMotor armMotor1 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final EV3LargeRegulatedMotor armMotor2 = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	//sensor ports


	//robot dimension constants
	public static final double ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR = 10.6;
	public static final double WHEEL_RADIUS = 2.09;
	public static final double TRACK = 15.15; 
	
	public static void main(String[] args)  {
		/*
		 * FOLLOWING CODE NEED TO RUN TEST, DO NOT DELETE
		 * 
		//WiFi module
		//Set up WiFi connection, require data from server, parse data and disconnect from server.
		WifiConnection conn = null;
		
		//while(true){
			try {
				conn = new WifiConnection(SERVER_IP, TEAM_NUMBER);
			} catch (IOException e) {
				LCD.drawString("Connection failed", 0, 1);
			}
		//}
		//Data received from the server is saved in "t".  
		Transmission t = conn.getTransmission();
		//Display the data in t
		if (t == null) {
			LCD.drawString("Failed to read transmission", 0, 5);
		} else {
			conn.printTransmission();
		}
		
		Button.waitForAnyPress();
		LCD.clear();
		*
		*/
		
		//*******************WiFi module ends**********************//
		
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
//		BlockDetector blockDetector = new BlockDetector(colorValue, colorData);
//		blockDetector.start();	
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
			// perform the ultrasonic localization
			//Rising edge was found to be the best in this case! So we use that one.
			//disable the side sensor for localization so that it doens't interfere
//			sidePoller.disableSensor();
			navi.setCmError(0.4);
			navi.setDegreeError(4.0);
			usl.doLocalization();
			
			navi.setCmError(0.2);
			navi.setDegreeError(2.0);
			lsl.doLocalization();
			
//			sidePoller.enableSensor();
			//double[] pos = {0, 0,0};
			//boolean[] up = {true,true,true};
			//odo.setPosition(pos,up);
			navi.travelToAndAvoid(150, 150);
			//leftMotor.setSpeed(10);
			//leftMotor.forward();
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);	
		
	}

}
