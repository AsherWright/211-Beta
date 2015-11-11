/*
 * LCDInfo.java
 * Alessandro Commodari and Asher Wright
 * ECSE 211 DPM Lab 4 - Localization
 * Group 53
 * This class controls the text that is displayed on the LCD screen. 
 * Taken from MyCourses.
 */
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
	// arrays for displaying data
	private double [] pos;
	
	public LCDInfo(Odometer odo, UltrasonicPoller frontPoller, UltrasonicPoller sidePoller, float[] usSideData) {
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		
		// initialise the arrays for displaying data
		pos = new double [3];
		this.frontPoller = frontPoller;
		this.sidePoller = sidePoller;
		
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
		LCD.drawString("H: ", 0, 2);
		LCD.drawString("F: ", 0, 3);
		LCD.drawString("S: ", 0, 4);
		LCD.drawString(String.valueOf(pos[0]), 3, 0);
		LCD.drawString(String.valueOf(pos[1]), 3, 1);
		LCD.drawString(String.valueOf(pos[2]), 3, 2);
		LCD.drawString(String.valueOf(frontPoller.getUsData()), 3, 3);
		LCD.drawString(String.valueOf(sidePoller.getUsData()), 3, 4);
	}

}
