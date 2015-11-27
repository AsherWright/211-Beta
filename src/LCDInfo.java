/*
 * LCDInfo.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 4 - Localization
 * Group 53
 * This class controls the text that is displayed on the LCD screen. 
 * Taken from MyCourses.
 */
import pollers.ColorSensorPoller;
import pollers.UltrasonicPoller;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Timer;
import lejos.utility.TimerListener;

public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private TextLCD LCD = LocalEV3.get().getTextLCD();
	private UltrasonicPoller frontPoller;
	private UltrasonicPoller sidePoller;
	private ColorSensorPoller blockPoller;
	private BlockDetector detector;
	// arrays for displaying data
	private double [] pos;
	
	public LCDInfo(Odometer odo, UltrasonicPoller frontPoller, UltrasonicPoller sidePoller,ColorSensorPoller blockPoller, BlockDetector detector) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		
		// initialise the arrays for displaying data
		pos = new double [3];
		this.frontPoller = frontPoller;
		this.sidePoller = sidePoller;
		this.blockPoller = blockPoller;
		this.detector = detector;
		// start the timer
		lcdTimer.start();
	}
	/*
	 * (non-Javadoc)
	 * @see lejos.utility.TimerListener#timedOut()
	 * Prints the info on the LCD.
	 */
	public void timedOut() {
		odo.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("T: ", 0, 2);
		LCD.drawString("F: ", 0, 3);
		LCD.drawString("S: ", 0, 4);
		LCD.drawString("C: ", 0, 5);
		LCD.drawString("B: ", 0, 6);
		LCD.drawString(String.valueOf(pos[0]), 3, 0);
		LCD.drawString(String.valueOf(pos[1]), 3, 1);
		LCD.drawString(String.valueOf(pos[2]), 3, 2);
		LCD.drawString(String.valueOf(frontPoller.getUsData()), 3, 3);

		LCD.drawString(String.valueOf(sidePoller.getUsData()), 3, 4);
		LCD.drawString(String.valueOf(blockPoller.getR()) + ", " + String.valueOf(blockPoller.getG()) + ", " + String.valueOf(blockPoller.getB()), 3, 5);
		LCD.drawString(detector.getBlockType(), 3, 6);

	}

}
