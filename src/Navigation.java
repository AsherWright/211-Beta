import pollers.UltrasonicPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * @author TEAM 14
 * @version 2.0
 * ECSE 211 CTF Robot
 * This class controls the navigation of the robot. It can travel to places,
 * travel to places while avoiding blocks, and rotate.
 * Originally written by: Sean Lawlor
 * Ported to EV3 by: Francois Ouellet Delorme
 */
public class Navigation {

	//global variables
	final static int ACCELERATION = 1000;
	final int ROTATION_SPEED_FOR_LIGHTLOCALIZATION = 180;
	private int fast;
	private int slow;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private double wheelRadius;
	private double wheelBase;
	private WallAvoider avoider;
	private UltrasonicPoller frontPoller;
	private double degreeError;
	private double cmError;
	
	/**
	 * 
	 * @param odo The odometer of the robot
	 * @param avoider The wallavoider of the robot
	 * @param frontPoller The frontfacing ultrasonic sensor
	 * @param wheelRadius The radius of the wheels of the robot
	 * @param wheelBase The distance between the two wheels
	 */
	public Navigation(Odometer odo, WallAvoider avoider,  UltrasonicPoller frontPoller, double wheelRadius, double wheelBase) {
		this.wheelRadius = wheelRadius;
		this.wheelBase = wheelBase;
		this.odometer = odo;
		this.avoider = avoider;
		this.frontPoller = frontPoller;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
		degreeError = 2.0;
		cmError = 0.5;
		fast = 160;
		slow = 100;
	}
	/**
	 * Sets the traveling forward speed of the robot
	 * @param speed How fast to turn the motors while traveling forward
	 */
	public void setFastSpeed(int speed){
		this.fast = speed;
	}
	/**
	 * Sets the rotational speed of the robot
	 * @param speed How fast to turn the motors while rotating
	 */
	public void setSlowSpeed(int speed){
		this.slow = speed;
	}
	/**
	 * Sets the amount of positional error the robot can be off the final position (in cm)
	 * @param cmErr The amount of error allowed
	 */
	public void setCmError(double cmErr){
		cmError = cmErr;
	}
	/**
	 * Sets the amount of degree error the robot can be off the final position (in deg)
	 * @param deg The amount of error allowed
	 */
	public void setDegreeError(double deg){
		degreeError = deg;
	}

	/**
	 * Sets the speeds of the motors.
	 * @param lSpd The speed to set the left motor
	 * @param rSpd The speed to set the right motor
	 */
	private void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
		
		if(lSpd == 0 && rSpd == 0){
			this.leftMotor.stop(true);
			this.rightMotor.stop(true);
		}
	}

	/**
	 * Function that makes the robot travel to a position in the arena
	 * @param x The x coordinate of the position
	 * @param y The y coordinate of the position
	 */
	public void travelTo(double x, double y) {
		double minAng;
		while (Math.abs(x - odometer.getX()) > cmError || Math.abs(y - odometer.getY()) > cmError) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(fast, fast);
			
		}
		this.setSpeeds(0, 0);
	}

	/*
	 * TravelToAndAvoid function which takes as arguments the x and y position in cm Will travel to designated position, while
	 * constantly updating it's heading. It also avoids blocks in the way
	 */
	/**
	 * Function that makes the robot travel to a position in the arena WHILE avoiding obstacles
	 * @param x The x coordinate of the position
	 * @param y The y coordinate of the position
	 */
	public void travelToAndAvoid(double x, double y) {
		double minAng;
		while (Math.abs(x - odometer.getX()) > cmError || Math.abs(y - odometer.getY()) > cmError) {
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			this.turnTo(minAng, false);
			this.setSpeeds(fast, fast);
			
			//if we see a block coming up, RUN wallFollower.avoidWall();
			if(frontPoller.getUsData() < 13){
				Sound.beep();
				Sound.beep();
				this.setSpeeds(0, 0);
				avoider.avoidWall(odometer.getX()+10.0*Math.cos(odometer.getAng()), odometer.getY()+10.0*Math.sin(odometer.getAng()),x, y);	
				this.setSpeeds(fast,fast);

			}
			
		}
		this.setSpeeds(0, 0);
	}
	

	/**
	 * Turns the robot to face a certain orientation.
	 * @param angle The angle to face the robot (0 deg along x axis)
	 * @param stop Whether to stop the robot after turning
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getAng();

		while (Math.abs(error) > degreeError) {

			error = angle - this.odometer.getAng();

			if (error < -180.0) {
				this.setSpeeds(-slow, slow);
			} else if (error < 0.0) {
				this.setSpeeds(slow, -slow);
			} else if (error > 180.0) {
				this.setSpeeds(slow, -slow);
			} else {
				this.setSpeeds(-slow, slow);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}

	/**
	 * Checks to see if either of the wheels is rotating
	 * @return True if rotating
	 */
	public boolean isRotating(){
		return rightMotor.isMoving() || leftMotor.isMoving();
	}
	/**
	 * Converts a distance to travel in the arena to an amount the robot's wheels should spin
	 * @param radius The radius of the wheels of the robot
	 * @param distance The distance the robot should travel
	 * @return The amount to spin the robot's wheels (in deg)
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * Converts an amount the robot should rotate into an amount to spin the wheels
	 * @param radius The radius of the robot's wheels
	 * @param width The distance between the two wheels of the robot
	 * @param angle The angle to rotate the robot
	 * @return The amount to spin each wheel (in deg)
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This makes the robot rotate 360 degrees, and is used for light localization
	 */
	public void rotateForLightLocalization(){
		leftMotor.setSpeed(ROTATION_SPEED_FOR_LIGHTLOCALIZATION);
		rightMotor.setSpeed(ROTATION_SPEED_FOR_LIGHTLOCALIZATION);
		leftMotor.rotate(-convertAngle(wheelRadius, wheelBase, 360.0), true);
		rightMotor.rotate(convertAngle(wheelRadius, wheelBase, 360.0), true);
	}
}
