package pollers;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This is a "Ultrasonic Poller". It controls an ultrasonic sensor. It can control
 * how often it gets data, whether or not it is enabled, etc. It is used to get data
 * from ultrasonic sensors
 * @author AsherW
 *
 */
public class UltrasonicPoller extends Thread{

	private Port usPort;
	EV3UltrasonicSensor usSensor;		// usSensor is the instance
	SampleProvider usDistance;	// usDistance provides samples from this instance
	float[] usData;		// usData is the buffer in which data are returned
	private Object lock;
	private int pollPeriod;
	/**
	 * Basic constructor
	 * @param portName The port that the sensor is plugged into the robot
	 */
	public UltrasonicPoller(String portName) {
		usPort = LocalEV3.get().getPort(portName);
  		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		usSensor =  new EV3UltrasonicSensor(usPort);
  		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		usDistance = usSensor.getMode("Distance");
		usData =  new float[usDistance.sampleSize()];
		lock = new Object();
		pollPeriod = 50;
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.start();
	}
	/**
	 * The run method (overrides Thread's run method). 
	 */
	public void run() {
		
		while (true) {
			synchronized(lock){
				if(usSensor.isEnabled()){
				usDistance.fetchSample(usData,0);							// acquire data
				}
			}	
			try { Thread.sleep(pollPeriod); } catch(Exception e){}		// Poor man's timed sampling
		}
	}
	/**
	 * Disables the ultrasonic sensor
	 */
	public void disableSensor(){
		synchronized (lock) {
			usSensor.disable();			
		}
	}
	/**
	 * Enables the ultrasonic sensor
	 */
	public void enableSensor(){
		synchronized (lock) {
			usSensor.enable();			
		}
	}
	/**
	 * Gets the distance measurement from the ultrasonic sensor
	 * @return The distance the ultrasonic sensor is reading
	 */
	public float getUsData() {
		synchronized (lock) {
			return (float) (usData[0]*100.0);
		}
	}


	/**
	 * Sets the rate at which the sensor collects data
	 * @param pollRate The period of the data collection
	 */
	public void setPollRate(int pollPeriod){
		synchronized (lock){
			this.pollPeriod = pollPeriod;
		}

	}

}