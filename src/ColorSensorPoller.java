import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

public class ColorSensorPoller extends Thread {
	private Port csPort;
	EV3ColorSensor csSensor;		// usSensor is the instance
	SampleProvider csSample;	// usDistance provides samples from this instance
	Port colorPort;
	float[] csData;		// usData is the buffer in which data are returned
	private Object lock;
	
	
	public ColorSensorPoller(String port){
		csPort = LocalEV3.get().getPort(port);		
  		csSensor = new EV3ColorSensor(csPort);
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
	 * From the reference of EV3 web, color sensor sample rate is 1kHz, therefore the minimum time between each data is 1ms.
	 * We fetchSample every 10ms. 
	 */
	public void run() {
		
		while (true) {
			synchronized(lock){
				csSample.fetchSample(csData, 0);                    // acquire data
			}	
			try { Thread.sleep(20); } catch(Exception e){}		// Poor man's timed sampling
		}
	}
	public float[] getColorData(){
		synchronized(lock){
			return csData;
		}		
	}
	public double getR(){
		synchronized(lock){
			return csData[0];
		}		
	}
	public double getG(){
		synchronized(lock){
			return csData[1];
		}		
	}
	public double getB(){
		synchronized(lock){
			return csData[2];
		}		
	}
	public double getBrightness(){
		synchronized (lock) {
			return csData[0];
			
		}		
	}
}
