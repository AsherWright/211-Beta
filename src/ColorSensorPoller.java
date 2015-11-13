import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class ColorSensorPoller extends Thread {
	private Port csPort;
	EV3ColorSensor csSensor;		// usSensor is the instance
	SampleProvider csSample;	// usDistance provides samples from this instance
	Port colorPort;
	float[] csData;		// usData is the buffer in which data are returned
	private Object lock;
	private double r,g,b;
	
	public ColorSensorPoller(String port){
		colorPort = LocalEV3.get().getPort(port);		
  		csSensor = new EV3ColorSensor(colorPort);
  		csSample = csSensor.getRGBMode();	// colorValue provides samples from this instance
		csData = new float[csSample.sampleSize()];			// colorData is the buffer in which data are returned
		lock = new Object();
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.start();
	}
	public void setMode(int mode){
		if(mode ==1){
			csSample = csSensor.getRedMode();
			csData = new float[csSample.sampleSize()];
		}else if(mode ==2){
			csSample = csSensor.getRGBMode();
			csData = new float[csSample.sampleSize()];
		}
	}
	public void run() {
		
		while (true) {
			synchronized(lock){
				csSample.fetchSample(csData, 0);							// acquire data
			}	
			try { Thread.sleep(50); } catch(Exception e){}		// Poor man's timed sampling
		}
	}
	public float[] getColorData(){
		return csData;
	}
	public double getR(){
		return csData[0];
	}
	public double getG(){
		return csData[1];
	}
	public double getB(){
		return csData[2];
	}
	public double getBrightness(){
		return csData[0];
	}
}
