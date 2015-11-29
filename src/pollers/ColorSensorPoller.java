package pollers;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
/**
 * This is a "Color Sensor Poller". It controls a color sensor. It can control
 * how often it gets data, whether or not it is enabled, which color mode is set, etc.
 * It is used to get data from color sensors
 * @author AsherW
 *
 */
public class ColorSensorPoller extends Thread {
	private Port csPort;
	EV3ColorSensor csSensor;		// usSensor is the instance
	SampleProvider csSample;	// usDistance provides samples from this instance
	Port colorPort;
	float[] csData;		// usData is the buffer in which data are returned
	private Object lock;
	int pollRate;
	/** 
	 * Basic constructor
	 * @param port The port in which the color sensor is plugged into the robot
	 */
	public ColorSensorPoller(String port){
		csPort = LocalEV3.get().getPort(port);
  		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
  		csSensor = new EV3ColorSensor(csPort);
  		try {
			Thread.sleep(200);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
  		csSample = csSensor.getRGBMode();	// colorValue provides samples from this instance
		csData = new float[csSample.sampleSize()];			// colorData is the buffer in which data are returned
		lock = new Object();
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		pollRate = 30;
		this.start();
	}
	/**
	 * Sets the mode of the color sensor. 
	 * @param mode The mode to set the color sensor to. 1 = red, 2 = RGB
	 */
	public void setMode(int mode){
		synchronized(lock){
			if(mode ==1){
			csSensor.setCurrentMode(1); //1 = red
			csSample = csSensor.getRedMode();
			csData = new float[csSample.sampleSize()];
		}else if(mode ==2){
			csSensor.setCurrentMode("RGB");
			csSample = csSensor.getRGBMode();
			csData = new float[csSample.sampleSize()];
		}
	}	
	}
	/**
	 * The run method (overrides Thread's run method). 
	 */
	public void run() {
		
		while (true) {
			synchronized(lock){
				csSample.fetchSample(csData, 0);                    // acquire data
			}	
			try { Thread.sleep(pollRate); } catch(Exception e){}		// Poor man's timed sampling
		}
	}
	/**
	 * Gets the color data from the sensor 
	 * @return The color data in an array (different size and format depending on mode)
	 */
	public float[] getColorData(){
		synchronized(lock){
			return csData;
		}		
	}
	/**
	 * Gets the red value of the color sensor
	 * @return The red reading of the color sensor
	 */
	public double getR(){
		synchronized(lock){
			return csData[0];
		}		
	}
	/**
	 * Gets the green value of the color sensor
	 * @return The green reading of the color sensor
	 */
	public double getG(){
		synchronized(lock){
			return csData[1];
		}		
	}
	/**
	 * Gets the Blue value of the color sensor
	 * @return The blue reading of the color sensor
	 */
	public double getB(){
		synchronized(lock){
			return csData[2];
		}		
	}
	/**
	 * Gets the brightness read by the color sensor
	 * @return The brightness read by the color sensor
	 */
	public double getBrightness(){
		synchronized (lock) {
			return csData[0];
			
		}		
	}
	/**
	 * Sets the rate at which the sensor collects data
	 * @param pollRate The period of the data collection
	 */
	public void setPollRate(int pollRate){
		synchronized (lock) {
			this.pollRate = pollRate;
		}
	}
}
