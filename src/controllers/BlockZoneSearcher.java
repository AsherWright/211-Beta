package controllers;

import odometry.Odometer;
import pollers.UltrasonicPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * @author danielebercovici
 * @version v1.1
 */
public class BlockZoneSearcher {
	
	//get incoming values for variables
	private Odometer odo;
	private Navigation navi;
	private BlockDetector detector;
	private UltrasonicPoller frontPoller;
	private UltrasonicPoller sidePoller;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private double[] pos = new double [3];
	private boolean isFlag; //determines if flag is found
	private double b = 6; //perimeter 
	private double a = 3; //start position
	
	/**
	 * Class Constructor
	 * @param sidePoller USsensor on the side of robot
	 * @param frontPoller USsensor on the front of robot
	 * @param navi Navigator class Object
	 * @param odo Odometer class Object
	 * @param detector BlockDetector class Object
	 */
	public BlockZoneSearcher(UltrasonicPoller sidePoller,UltrasonicPoller frontPoller, Navigation navi, Odometer odo,BlockDetector detector)

	{
        this.odo = odo;
        this.navi= navi;
		EV3LargeRegulatedMotor[] motors = odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
        this.detector= detector;
        this.frontPoller = frontPoller;
        this.sidePoller = sidePoller;
	}

	/**
	 * Runs the Search algorithm of searching for the flag along the perimeter of the opponents zone and exiting when flag is found 
	 */
	public void searchZone(int startX, int startY, int zoneWidth, int zoneHeight, String direction){
		/*
		 * we have to figure out if we are going up first (up) or down first (down), and adjust 
		 * signs accordingly.
		 */
		int factor = 1;
		double heightOfZoneAndBuffer = zoneHeight*30.4+a+b;
		double widthOfZoneAndBuffer = zoneWidth*30.4 + b;
		
		if(direction.equals("down")){
			factor = 1;
		}else if(direction.equals("up")){
			factor = -1;
		}
		//turn off front sensor
		frontPoller.disableSensor();
		isFlag = detector.isFlag(); //false
		
		rightMotor.setSpeed(150);
		leftMotor.setSpeed(150);
		//our next Y position is our startY either + or - the height of the zone.
		double nextYPosition = startY*30.4 - factor*(heightOfZoneAndBuffer);
		//travel side to upper right corner
		while((odo.getY() - nextYPosition)*factor > 0)
		{	
			if(sidePoller.getUsData() < (b+30.4))//check for objects in first 1x3 section
			{
				checkObject();
				if (isFlag)
				{
					break;
				}
			}
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);
			rightMotor.forward();
			leftMotor.forward();

		}
		
		if(!isFlag)//didnt find flag in first side
		{
			//turn 90 degrees ccw
			rightMotor.rotate(convertAngle(Controller.WHEEL_RADIUS,Controller.TRACK,90), true);
			leftMotor.rotate(convertAngle(Controller.WHEEL_RADIUS,Controller.TRACK,-90), false);
					
			double nextXPosition = startX*30.4 + factor*widthOfZoneAndBuffer;
			//travel side 
			while(factor*(odo.getX() - nextXPosition) < 0) 
			{
				leftMotor.setSpeed(150);
				rightMotor.setSpeed(150);
				rightMotor.forward();
				leftMotor.forward();

			}
			//turn 90 degree ccw
			rightMotor.rotate(convertAngle(Controller.WHEEL_RADIUS,Controller.TRACK,90), true);
			leftMotor.rotate(convertAngle(Controller.WHEEL_RADIUS,Controller.TRACK,-90), false);
			
			while (factor*(odo.getY() - startY*30.4) < 0) //object in second 1x3 area
			{
				if(sidePoller.getUsData()< (b+30.4))
				{
					checkObject();
					if (isFlag)
					{
						break;
					}
				}
				leftMotor.setSpeed(150);
				rightMotor.setSpeed(150);
				rightMotor.forward();
				leftMotor.forward();
			}
			leftMotor.stop(true);
			rightMotor.stop(true);
		}
	}
	/**
	 * Gets near the block it found and switches to the front sensor, 
	 * calls detector to investigate block, moves on if not flag or returns if flag found
	 */
	private void checkObject()
	{
		Sound.beep();
		//move forward 7cm to approximately get to the middle of block 
		rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS,9), true);
		leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS,9), false);

		//record initial position
		odo.getPosition(pos);
		
		
		//turn off side sensor and turn on front sensor
		sidePoller.disableSensor();
		frontPoller.enableSensor();
		
		//investigate the block
		detector.investigateBlock();
		//Sound.buzz();
		isFlag = detector.isFlag();

		if (isFlag)
		{  
			return;
		}
		else{
		//turn off front sensor and turn on side sensor
		frontPoller.disableSensor();
		sidePoller.enableSensor();
		
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		
		//go back to original position
		navi.turnTo(pos[2], true);

		while (sidePoller.getUsData() <= (b+30.4)) //travel into cant see block anymore
			{
				leftMotor.setSpeed(150);
				rightMotor.setSpeed(150);
				rightMotor.forward();
				leftMotor.forward();
			}
//		try {
//			Thread.sleep(300);
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
		Sound.beep();
		rightMotor.rotate(convertDistance(Controller.WHEEL_RADIUS,8), true);
		leftMotor.rotate(convertDistance(Controller.WHEEL_RADIUS,8), false);
		
		
		}
	}
	/**
	 * Converts a distance in cm to angle in degrees
	 * @param radius the wheel radius
	 * @param distance desired distance to convert
	 * @return angle in degrees
	 */
	private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
    
    /**
     * Converts an angle to distance in cm
     * @param radius the wheel radius
     * @param width the bandwidth of the two wheels 
     * @param angle desired angle to convert
     * @return result of method convertDistance() distance in cm
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }

}
