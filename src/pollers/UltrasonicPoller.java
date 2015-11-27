package pollers;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

//create this thread only for polling data from US sensor every 50ms
public class UltrasonicPoller extends Thread{

	private Port usPort;
	EV3UltrasonicSensor usSensor;		// usSensor is the instance
	SampleProvider usDistance;	// usDistance provides samples from this instance
	float[] usData;		// usData is the buffer in which data are returned
	private Object lock;
	private int pollPeriod;
	public UltrasonicPoller(String portName) {
		usPort = LocalEV3.get().getPort(portName);
		usSensor =  new EV3UltrasonicSensor(usPort);
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
	public void disableSensor(){
		synchronized (lock) {
			usSensor.disable();			
		}
	}
	public void enableSensor(){
		synchronized (lock) {
			usSensor.enable();			
		}
	}
//  Sensors now return floats using a uniform protocol.	
	public float getUsData() {
		synchronized (lock) {
			return (float) (usData[0]*100.0);
		}
	}

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
	public void setPollRate(int pollPeriod){
		synchronized (lock){
			this.pollPeriod = pollPeriod;
		}

	}

}