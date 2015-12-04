package controllers;
import java.io.File;




import java.io.IOException;

import odometry.Odometer;
import odometry.OdometerCorrector;
import pollers.ColorSensorPoller;
import pollers.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import localization.LightLocalizer;
import localization.USLocalizer;
import localization.USLocalizer.LocalizationType;
import view.LCDDisplay;
import wifi.StartCorner;
import wifi.Transmission;
import wifi.WifiConnection;
/**
 * @author Asher Wright
 * @version 2.0
 * ECSE 211 CTF Robot
 * This class sets up the classes for Finding the objects, and calls them. It also initializes the sensors
 * and the motors.
 */
public class Controller {
	//variables for WiFi module 
	// *** INSTRUCTIONS ***
	// SERVER_IP: the IP address of the computer running the server application
	private static final String SERVER_IP ="192.168.10.200"; //"192.168.10.19";//"192.168.10.200";//"192.168.10.116";//"192.168.10.120";//"192.168.10.116"; //YAN or Rahul: "192.168.43.118";
	private static final int TEAM_NUMBER = 14;	

	//robot dimension constants
	public static final double ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR = 10.6;
	public static final double WHEEL_RADIUS = 2.09;
	public static final double TRACK = 15.15; 
	public static final double TILE_WIDTH = 30.4;

	// Static Resources:
	//LCD screen
	private static TextLCD LCD = LocalEV3.get().getTextLCD();
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
	

	public static void main(String[] args)  {

		UltrasonicPoller frontPoller = new UltrasonicPoller("S4");
		UltrasonicPoller sidePoller = new UltrasonicPoller("S1");
		ColorSensorPoller blockPoller = new ColorSensorPoller("S2");
		ColorSensorPoller groundPoller = new ColorSensorPoller("S3");
		groundPoller.setMode(1);
		// start the block detector thread, which will be constantly checking with the light sensor
		//to see if there is a block.
		boolean wifiWorked = true;
		
		//*********************************WiFi module************************************//
	    //Set up WiFi connection, require data from server, parse data and disconnect from server.
		WifiConnection conn = null;
		Transmission t = null;
		try {
			conn = new WifiConnection(SERVER_IP, TEAM_NUMBER);
			} catch (IOException e) {
			LCD.drawString("Connection failed", 0, 1);
			wifiWorked = false;
		}		
		if(conn == null){
			LCD.drawString("Unable to find Server", 0, 5);
			wifiWorked = false;
		}else{
			//Data received from the server is saved in "t". 
			//Pass the data saved in t to the relevant class
			t = conn.getTransmission();
			//Display the data in t
			if (t == null) {
				LCD.drawString("Failed to read transmission", 0, 5);
				wifiWorked = false;
			} else {
				conn.printTransmission();
			}
			//Button.waitForAnyPress();
			LCD.clear();
			
		}
		//********************************WiFi module ends******************************//
		
		if(wifiWorked){	
			//variables from WIFI
			int flagType = t.flagType;
			StartCorner startingCorner = t.startingCorner;
			int bottomLeftX = t.opponentHomeZoneBL_X;
			int bottomLeftY = t.opponentHomeZoneBL_Y;
			int topRightX = t.opponentHomeZoneTR_X;
			int topRightY = t.opponentHomeZoneTR_Y;
			int capturePointX = t.dropZone_X;
			int capturePointY = t.dropZone_Y;
			//variables HARDCODED
//			int flagType = 1;
//			StartCorner startingCorner = StartCorner.TOP_RIGHT;
//			int bottomLeftX = 3;
//			int bottomLeftY = 3;
//			int topRightX = 5;
//			int topRightY = 5;
//			int capturePointX = 1;
//			int capturePointY = 1;

			//***********************Initialization Module******************************//
			// setup the odometer
			Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
			OdometerCorrector odoCorr = new OdometerCorrector(odo, groundPoller);
			
			//setup the wall avoider
			WallAvoider avoider = new WallAvoider(odo, frontPoller, sidePoller);
			
			//set up the navigator
			Navigation navi = new Navigation(odo, avoider, frontPoller, WHEEL_RADIUS, TRACK);
			
			//set up the detector
			BlockDetector blockDetector = new BlockDetector(blockPoller, navi, odo, frontPoller, verticalArmMotor, horizontalArmMotor, flagType);
			blockDetector.start();	
			//set up the searcher
			BlockZoneSearcher flagSearcher = new BlockZoneSearcher(sidePoller, frontPoller, navi, odo, blockDetector);
			//set up the LCD
			LCDDisplay lcd = new LCDDisplay(odo,frontPoller,sidePoller, blockPoller, blockDetector);
			lcd.start();
			
			//set up the localization
			LightLocalizer lsl = new LightLocalizer(odo, groundPoller, navi, ROBOT_CENTRE_TO_LIGHTLOCALIZATION_SENSOR, startingCorner);
			USLocalizer usl = new USLocalizer(odo,navi, frontPoller, USLocalizer.LocalizationType.FULL_CIRCLE);
			//***********************End of Initialization******************************//

			
			double angleForSearch;
			String searchDirection = "";
			int searchStartX;
			int searchStartY;
			int firstCornerX;
			int firstCornerY;
			double zoneBuffer;
			// we use our starting corner to determine where we want to travel to in order to search. 
			if(startingCorner == StartCorner.BOTTOM_LEFT || startingCorner == StartCorner.BOTTOM_RIGHT){
				searchDirection = "down";
				searchStartX = bottomLeftX;
				searchStartY = topRightY;
				firstCornerX = bottomLeftX;
				firstCornerY = bottomLeftY;
				zoneBuffer = -1*TILE_WIDTH/3.0;
				angleForSearch = 270;

			}else{ //it is the top left or top right
				searchDirection = "up";
				searchStartX = topRightX;
				searchStartY = bottomLeftY;
				firstCornerX = topRightX;
				firstCornerY = topRightY;
				zoneBuffer = TILE_WIDTH/3.0;
				angleForSearch = 90;

			}
			
			/*
			 * Step 1: Ultrasonic Localization
			 * 	Figure out where we are, roughly
			 */
			//disable the side sensor for localization so that it doesn't interfere
			sidePoller.disableSensor();
			//set the errors on the navigation to be large for US localization
			navi.setCmError(0.5);
			navi.setDegreeError(4.0);
			//perform ultra-sonic localization
			usl.doLocalization();
			
			/*
			 * Step 2: Light Localization
			 * 	Figure out where we are, precisely
			 */
			//set the errors back to smaller values
			navi.setCmError(0.5);
			navi.setDegreeError(2.0);
			//perform light-sensor localization
			lsl.doLocalization();
			
			
			/*
			 * Step 3: Travel to first corner
			 * 	Travel to the corner of the block zone in which we are going to relocalize
			 */
			//enable the side poller for navigating
			sidePoller.enableSensor();
			//speed up the robot for navigation
			navi.setSlowSpeed(90);
			navi.setFastSpeed(160);
			//start odometry correction
			odoCorr.start();
			//navigation
			navi.travelToAndAvoid(TILE_WIDTH*firstCornerX, TILE_WIDTH*firstCornerY);
			
			/*
			 * Step 4: Relocalize
			 * 	Use the lines to relocalize (figure out where we are, again)
			 */
			//perform second localization
			lsl.doRelocalization(TILE_WIDTH*firstCornerX, TILE_WIDTH*firstCornerY);
			
			/*
			 * Step 5: Travel to second corner & rotate
			 * 	Travel to the top (or bottom) corner of the block zone, and turn to searching position
			 */
			//we first travel a bit away from the zone
			navi.travelTo(TILE_WIDTH*firstCornerX + zoneBuffer, TILE_WIDTH*firstCornerY + zoneBuffer);
			//we travel to the second corner with no avoidance (there can't be any blocks there, anyway)
			navi.travelTo(TILE_WIDTH*searchStartX + zoneBuffer, TILE_WIDTH*searchStartY);
			navi.turnTo(angleForSearch, true);
			
			/*
			 * Step 6: Search for block
			 * 	Start to search around the perimeter for the block
			 */	
			//lower errors
			navi.setCmError(0.4);
			navi.setDegreeError(3);
			//find dimensions of place to search
			int zoneWidth = topRightX - bottomLeftX;
			int zoneHeight = topRightY - bottomLeftY;
			//run searcher
			flagSearcher.searchZone(searchStartX, searchStartY, zoneWidth, zoneHeight, searchDirection);
			
			/*
			 * Step 7: Drive to end position
			 * 	We now have the block. Drive to the final position 
			 */
			if(blockDetector.isFlag()){
				//we want to travel to the center of the blocks.
				navi.travelToAndAvoid(capturePointX*TILE_WIDTH+TILE_WIDTH/2, capturePointY*TILE_WIDTH+TILE_WIDTH/2); //wifi
			}
			
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			System.exit(0);	
		}
	}
}