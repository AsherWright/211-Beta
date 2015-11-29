import lejos.utility.Timer;
import lejos.utility.TimerListener;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * 
 * @author TEAM 14
 * Originally written by: Sean Lawlor
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * ECSE 211 - DPM: CTF Robot
 * 
 * Class which controls the odometer for the robot
 * 
 * Odometer defines cooridinate system as such...
 * 
 * 					90Deg:pos y-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 180Deg:neg x-axis------------------0Deg:pos x-axis
 * 							|
 * 							|
 * 							|
 * 							|
 * 					270Deg:neg y-axis
 * 
 * The odometer is initalized to 90 degrees, assuming the robot is facing up the positive y-axis
 */
public class Odometer implements TimerListener {

	private Timer timer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private final int DEFAULT_TIMEOUT_PERIOD = 20;
	private double leftRadius, rightRadius, width;
	private double x, y, theta;
	private double[] oldDH, dDH;
	
	/**
	 * Basic Constructor
	 * @param leftMotor The left motor of the robot
	 * @param rightMotor The right motor of the robot
	 * @param INTERVAL The interval to update the odometer
	 * @param autostart Whether or not to start the odometer right away
	 */
	public Odometer (EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int INTERVAL, boolean autostart) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
		// default values, modify for your robot
		this.rightRadius = Controller.WHEEL_RADIUS;
		this.leftRadius = Controller.WHEEL_RADIUS;
		this.width = Controller.TRACK;
		
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0;
		this.oldDH = new double[2];
		this.dDH = new double[2];

		if (autostart) {
			// if the timeout interval is given as <= 0, default to 20ms timeout 
			this.timer = new Timer((INTERVAL <= 0) ? INTERVAL : DEFAULT_TIMEOUT_PERIOD, this);
			this.timer.start();
		} else
			this.timer = null;
	}
	
	/**
	 * Stops the odometer
	 */
	public void stop() {
		if (this.timer != null)
			this.timer.stop();
	}
	/**
	 * Starts the odometer
	 */
	public void start() {
		if (this.timer != null)
			this.timer.start();
	}
	

	/**
	 * Calculates displacement and heading
	 * @param data The wheel data of the robot
	 */
	private void getDisplacementAndHeading(double[] data) {
		int leftTacho, rightTacho;
		leftTacho = leftMotor.getTachoCount();
		rightTacho = rightMotor.getTachoCount();

		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) * Math.PI / 360.0;
		data[1] = (rightTacho * rightRadius - leftTacho * leftRadius) / width;
	}

	/**
	 * Recompute the odometer values using the displacement and heading changes
	 */
	public void timedOut() {
		this.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (this) {
			theta += dDH[1];
			theta = fixDegAngle(theta);

			x += dDH[0] * Math.cos(Math.toRadians(theta));
			y += dDH[0] * Math.sin(Math.toRadians(theta));
		}

		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	/**
	 * 
	 * @return The x coordinate of the robot
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	/**
	 * 
	 * @return The y coordinate of the robot
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	/**
	 * 
	 * @return The Angle (orientation) of the robot
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}

	/**
	 * Sets the position and orientation of the robot
	 * @param position The position of the robot in form [x,y,theta]
	 * @param update Which variables to update in form [boolean,boolean,boolean]
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (this) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}
	/**
	 * Sets the angle (orientation) of the robot
	 * @param angle The orientation to set the robot
	 */
	public void setTheta(double angle){
		synchronized(this){
			theta = angle;
		}
	}
	/**
	 * Gives the position of the robot via a passed in array
	 * @param position The robot's position in form [x,y,theta]
	 */
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta;
		}
	}
	/**
	 * Gives the position of the robot via returning a new position array in form [x,y,theta]
	 * @return The position of the robot in form [x,y,theta]
	 */
	public double[] getPosition() {
		synchronized (this) {
			return new double[] { x, y, theta };
		}
	}
	
	/**
	 * Gets the robot's two motors
	 * @return The two motors of the robot in an array form: [left motor, right motor]
	 */
	public EV3LargeRegulatedMotor [] getMotors() {
		return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}
	/**
	 * Get's the robot's left motor
	 * @return The left motor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	/**
	 * Get's the robot's right motor
	 * @return The right motor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}

	/**
	 * Fixes the angle of the robot my moding it by 360. Performs wrapping.
	 * @param angle The angle of the robot
	 * @return The fixed angle
	 */
	private static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}

}
