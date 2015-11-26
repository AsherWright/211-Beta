/*
 * Beta demo.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 5 - Finding Objects
 * Group 53
 * This class sets up the classes for Finding the objects, and calls them. It also initializes the sensors
 * and the motors.
 */

import java.io.File;
import java.io.IOException;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import wifi.StartCorner;
import wifi.Transmission;
import wifi.WifiConnection;

public class Controller {
	//variables for WiFi module 
	// *** INSTRUCTIONS ***
	// SERVER_IP: the IP address of the computer running the server application
	private static final String SERVER_IP = "192.168.10.200";//"192.168.10.116";//"192.168.10.120";//"192.168.10.116"; //YAN or Rahul: "192.168.43.118";
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

//		UltrasonicPoller frontPoller = new UltrasonicPoller("S4");
		UltrasonicPoller frontPoller = new UltrasonicPoller("S4");
		UltrasonicPoller sidePoller = new UltrasonicPoller("S1");
		ColorSensorPoller blockPoller = new ColorSensorPoller("S2");
		ColorSensorPoller groundPoller = new ColorSensorPoller("S3");
		groundPoller.setMode(1);
		// start the block detector thread, which will be constantly checking with the light sensor
		//to see if there is a block.
		
		//************************WiFi module********************************//
	    //Set up WiFi connection, require data from server, parse data and disconnect from server.
//		WifiConnection conn = null;
//		try {
//			conn = new WifiConnection(SERVER_IP, TEAM_NUMBER);
//			} catch (IOException e) {
//			LCD.drawString("Connection failed", 0, 1);
//		}		
//		if(conn == null){
//			LCD.drawString("Unable to find Server", 0, 5);
//		}else{
//		//Data received from the server is saved in "t". 
//		//Pass the data saved in t to the relevant class
//		Transmission t = conn.getTransmission();
//		//Display the data in t
//		if (t == null) {
//			LCD.drawString("Failed to read transmission", 0, 5);
//		} else {
//			conn.printTransmission();
//		}
//			LCD.clear();
			//*******************WiFi module ends**********************//
	
			// setup the odometer
			Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
			OdometerCorrection odoCorr = new OdometerCorrection(odo, groundPoller);
			odoCorr.start();
			
			//setup the wall avoider
			WallAvoider avoider = new WallAvoider(odo, frontPoller, sidePoller);
	//		WallAvoider avoider = new WallAvoider(odo, frontPoller, null);
			//set up the display and navigator
			Navigation navi = new Navigation(odo, avoider, frontPoller, WHEEL_RADIUS, TRACK);
			
			//set up the localization
			BlockDetector blockDetector = new BlockDetector(blockPoller, navi, odo, frontPoller, verticalArmMotor, horizontalArmMotor, 5);
//		BlockDetector blockDetector = new BlockDetector(blockPoller, navi, odo, frontPoller, verticalArmMotor, horizontalArmMotor, t.flagType);
//			BlockDetector blockDetector = new BlockDetector(blockPoller, navi, odo, frontPoller, verticalArmMotor, horizontalArmMotor, 1);
			blockDetector.start();	
			//SearchingField searcher = new SearchingField(leftMotor, rightMotor, sidePoller, frontPoller, navi, odo, blockDetector, t.opponentHomeZoneBL_X, t.opponentHomeZoneTR_X);
	
			LCDInfo lcd = new LCDInfo(odo,frontPoller,sidePoller, blockPoller, blockDetector);
			
			//LCDInfo lcd = new LCDInfo(odo,frontPoller,sidePoller, blockPoller, blockDetector);
			//set up the light localization
	
	
			//set up the localization
	//		LightLocalizer lsl = new LightLocalizer(odo, groundPoller, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR);
			LightLocalizer lsl = new LightLocalizer(odo, groundPoller, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR, StartCorner.BOTTOM_LEFT);
//		    LightLocalizer lsl = new LightLocalizer(odo, groundPoller, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR, t.startingCorner);
			USLocalizer usl = new USLocalizer(odo,navi, frontPoller, USLocalizer.LocalizationType.FULL_CIRCLE);
			
			/*
			 * We wait for a press. If it is a left button, we're just doing the detection
			 * otherwise we do the block stuff.
			 */
			int buttonPressed = Button.waitForAnyPress();
			/** * ** *** **** ***** ****** ******* ******** ********* **********
			 * Left button means just do localization.
			 * Right button means do go capture the block and stay there
			 * Down button means Search
			 * Any other button means play shake it off. 
			 */
			if(buttonPressed == Button.ID_LEFT){
				sidePoller.disableSensor();
				navi.setCmError(0.4);
				navi.setDegreeError(4.0);
				usl.doLocalization();
				
				navi.setCmError(0.2);
				navi.setDegreeError(2.0);
				
				lsl.doLocalization();
			}else if(buttonPressed == Button.ID_RIGHT){ 
				//disable the side sensor for localization so that it doens't interfere
				sidePoller.disableSensor();
				//set the errors on the navigation to be large for US localization
				navi.setCmError(0.4);
				navi.setDegreeError(4.0);
				//perform ultrasonic localization
				usl.doLocalization();
				
				//we want small errors for the light sensor localization
				navi.setCmError(0.5);
				navi.setDegreeError(2.0);
				//perofrm lightsensor localization
				lsl.doLocalization();
				
				
				//enable the side poller for navigating
				sidePoller.enableSensor();
				//speed up the robot for this part
				navi.setSlowSpeed(90);
				navi.setFastSpeed(140);
				//travel to the flag's zone
				navi.travelToAndAvoid(30.4*2, 30.4*2);
				lsl.reLocalization(30.4*2, 30.4*2);
				navi.travelTo(30.4*2-10, 30.4*2-10);
//				navi.travelToAndAvoid(30.4*t.opponentHomeZoneBL_X - 5, 30.4*t.opponentHomeZoneBL_Y-5);
//				navi.travelTo(30.4*t.opponentHomeZoneBL_X-5, 30.4*t.opponentHomeZoneTR_Y-5);
			
				navi.turnTo(270, true);
				
				//now search for blocks in the area
				//searcher.run();
				//travel home (not needed for Beta)
				//navi.travelToAndAvoid(0, 0);
	
			}else if(buttonPressed == Button.ID_DOWN){
				SearchingField searcher = new SearchingField(leftMotor, rightMotor, sidePoller, frontPoller, navi, odo, blockDetector, 3, 5);
				double[] pos = new double[3];
				boolean[] update = new boolean[3];
				update[0] = true;
				update[1] = true;
				update[2] = true;
				pos[0] = 3*30.4-5;
				pos[1] = 5*30.4-5;
				pos[2] = 270;
				odo.setPosition(pos, update);
				navi.setCmError(0.4);
				navi.setDegreeError(3);
				searcher.run();
			}else{
				Sound.beep();
				File shakeItOff = new File("ShakeItOff.wav");
				System.out.println(Sound.playSample(shakeItOff, 100));
				Sound.beep();
	
			}
	
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);	
		}
//	}
}