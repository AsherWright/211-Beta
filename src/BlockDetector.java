import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
/**
 * @author danielebercovici
 * @version 1.0
 */
public class BlockDetector extends Thread {
	//constants
	//class variables
	//these are the different block profiles. They are initialized in the constructor
	private double[] blueBlockReading;
	private double[] darkBlueBlockReading;
	//the error that R,G, B can be off for it to still consider it a certain object.
	private static final double DETECTIONTHRESHOLDERROR = 0.5;
	private static final double WHEEL_RADIUS = 2.1;
	//color sensor variables
	private SampleProvider colorSensor;
	private float[] colorData;
	//Reading properties
	private boolean isReadingBlock;
	private String blockType;
	
	//TODO: Daniele you have to organize this you're the worst
	Odometer odo;
	Navigation navi;
	private double[] pos = new double [3];
	EV3LargeRegulatedMotor leftMotor;
	EV3LargeRegulatedMotor rightMotor;
	private SampleProvider usSensor;
	private float[] usData;
	private final int DETECTIONTHRESHOLD = 3;
	private final double tolerance = 3;
	EV3LargeRegulatedMotor verticalArmMotor;
	EV3LargeRegulatedMotor horizontalArmMotor;
	double USDistance;

	/**
	 * Class constructor
	 * @param colorSensor sample provider for colored sensor
	 * @param colorData sample data for colored sensor
	 * @param navi Navigator class Object
	 * @param odo Odometer class Object
	 * @param leftMotor EV3 Motor on the left 
	 * @param rightMotor EV3 Motor on the right
	 * @param usSensor Sample provider for ultrasonic sensor
	 * @param usData Sample data for Ultrasonic sensor
	 * @param verticalArmMotor EV3 Motor for up/down movement of arm
	 * @param horizontalArmMotor EV3 Motor for open/close movement of arm
	 */
	public BlockDetector(SampleProvider colorSensor, float[] colorData, Navigation navi, Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,SampleProvider usSensor, float[] usData, EV3LargeRegulatedMotor verticalArmMotor, EV3LargeRegulatedMotor horizontalArmMotor) {
		//get incoming values for variables
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.odo = odo;
		this.navi= navi;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usData = usData;
		this.usSensor = usSensor;
		this.verticalArmMotor = verticalArmMotor;
		this.horizontalArmMotor = horizontalArmMotor;
		
		//initialize variables
		blockType = "";
		isReadingBlock = false;
		//initialize block profiles
		blueBlockReading = new double[3];
		darkBlueBlockReading = new double[3];
		blueBlockReading[0] = 0.95;
		blueBlockReading[1] = 1.4;
		blueBlockReading[2] = 1.15;
		darkBlueBlockReading[0] = 0.2;
		darkBlueBlockReading[1] = 0.5;
		darkBlueBlockReading[2] = 0.7;

		
	}
	
	/**
	 * Method that Checks if Block is the right color, set global variable isReadingBlock true if right block and false otherwise.
	 */
	public void run(){
		while(true){
			//gets the data from the color sensor.
			colorSensor.fetchSample(colorData, 0);
			
			//checks the reading and compares it to each profile.
			double[] BlueBlockError = new double[3];
			double totalNoObjectError = 0;
			double[] DarkBlueBlockError  = new double[3];
			//go through R,G,B
			for(int i = 0; i < 3; i++){
				BlueBlockError[i] = Math.abs(colorData[i]*10 - blueBlockReading[i]);
				totalNoObjectError += Math.abs(colorData[i]*10);
				DarkBlueBlockError[i] = Math.abs(colorData[i]*10 - darkBlueBlockReading[i]);
			}
			//If our reading is within the allowed number to consider it a blue block, update what it sees.
			if(totalNoObjectError < 0.25){
				blockType = "";
				isReadingBlock = false;
			}else if(BlueBlockError[0] < DETECTIONTHRESHOLDERROR && BlueBlockError[1] < DETECTIONTHRESHOLDERROR &&  BlueBlockError[2] < DETECTIONTHRESHOLDERROR ){
				blockType = "BLOCK";
				isReadingBlock =true;
			}else if(DarkBlueBlockError[0] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[1] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[2] < DETECTIONTHRESHOLDERROR){
				blockType = "BLOCK";
				isReadingBlock = true;
			}else{
				blockType = "NOT BLOCK";
				isReadingBlock = false;
			}
			//now sleep so that we don't call this too often.
			try {
				Thread.sleep(200);											// sleep for 200 mS
			} catch (Exception e) {
				System.out.println("Error: " + e.getMessage());
			}
		}
	}
	/**
	 * Method that gets close to block in order to get accurate readings for the light sensor, calls isFlagDetected method
	 */
	public void investigateBlock()
	{
		//record initial position
		odo.getPosition(pos);
		
		
			double lTheta; 
			double rTheta; 
			USDistance = getFilteredUSData();
			
			//turn left till cant see block, record angle
			while(USDistance <= 30)
			{
				USDistance = getFilteredUSData();
				rightMotor.setSpeed(100);
				rightMotor.forward();
				
			}
			rightMotor.stop();
			Sound.beep();
			lTheta = pos[2]-odo.getAng();//record angle (initial-current angle)
			//navi.travelTo(pos[0], pos[1]);//return to initial position TODO: check if this is needed
			navi.turnTo(pos[2], false);
			Sound.beep();
			
			//turn right till cant see block, record angle 
			while(USDistance <= 30) 
			{
				USDistance = getFilteredUSData();
				leftMotor.setSpeed(100);
				leftMotor.forward();	
			}
			leftMotor.stop();
			Sound.beep();
			rTheta = pos[2]-odo.getAng();
			//navi.travelTo(pos[0], pos[1]);//return to initial position
			navi.turnTo(pos[2], false); 
			Sound.beep();
			

			/*calculate position in relation to the block 
			 * 1: within center threshold, drive within 3cm detect if its good a) good: pickup b) bad: go back to original position
			 * 2: too far to right or left, hit block, (back up 3cm?) detect if it is good a) good: pickup b) bad: go back to original position
			 */
			double difference = rTheta - lTheta;
			
			if(Math.abs(difference) < tolerance)//within center tolerance
			{
				USDistance = getFilteredUSData();
				while(USDistance != DETECTIONTHRESHOLD) //get within reading range
				{
					rightMotor.forward();
					leftMotor.forward();
					USDistance = getFilteredUSData();
				}
				rightMotor.stop();
				leftMotor.stop();
				
				isFlagDetected();
			} 
			
			else //too far to right or left
			{
				//drive into the block to straighten it out
				//TODO: test 10cm
				rightMotor.rotateTo(distanceToDegrees(10),false);
				leftMotor.rotateTo(distanceToDegrees(10),false);
				
				//TODO: test light sensor back up 3cm
				rightMotor.rotateTo(distanceToDegrees(-DETECTIONTHRESHOLD),false);
				leftMotor.rotateTo(distanceToDegrees(-DETECTIONTHRESHOLD),false);
				isFlagDetected();
			}
			
		}
		
	/**
	 * Method determines whether to capture flag by checking value of isReadingBlock calling pickUp method is true or returning robot to initial position otherwise
	 */
	public void isFlagDetected()
	{
		if(isReadingBlock) //capture flag
		{
			pickUp();
		}
		else //not flag, return to initial position
		{
			navi.travelTo(pos[0], pos[1]);
			navi.turnTo(pos[3], true);
		}
	}
	
	/**
	 * Method picks up flag its facing
	 */
	public void pickUp() 
	{
		//back up to TODO: 20cm
		while(USDistance <= 20)
		{
			rightMotor.backward();
			leftMotor.backward();
			USDistance = getFilteredUSData();
		}
		//turn around
		rightMotor.rotate(180);;
		leftMotor.rotate(-180);
		
		//lower arms
		verticalArmMotor.setSpeed(100);
		verticalArmMotor.rotate(-180,false);
		try {
			Thread.sleep(2000);
		} 
		catch (Exception e) {
		}
		
		//open arm
		horizontalArmMotor.setSpeed(100);
		horizontalArmMotor.rotate(100, false); //TODO: does it need time to sleep of can it open up the same time?
		
		//back up 20+cm //TODO: test distance
		rightMotor.rotateTo(distanceToDegrees(-20),false);
		leftMotor.rotateTo(distanceToDegrees(-20),false);
		
		//close arms
		horizontalArmMotor.rotate(-75, false);
		Sound.beep();
		try {
			Thread.sleep(2000);
		} catch (Exception e) {	
		}
		
		//lift up arms
		verticalArmMotor.rotate(180,false);
	}
	
	/**
	 * The method converts distance in cm to angle degrees of rotation
	 * @param distance distance in cm you want to travel (negative for direction)
	 * @return angle in degrees needed to turn to travel given distance
	 */
	public int distanceToDegrees(double distance)
	{
		int theta = (int) (distance/WHEEL_RADIUS); //theta = s/r
		theta = (int) (theta *180/Math.PI); //convert radians to degrees
		return theta;
	}

	//accessors
	public float[] getColorData(){
		synchronized (this) {
			return colorData;	
		}
	}
	public String getBlockType(){
		synchronized (this) {
			return blockType;	
		}
	
	}
	public boolean isReadingBlock(){
		synchronized (this) {
			return isReadingBlock;	
		}
	}

	private float getFilteredUSData() {
		usSensor.fetchSample(usData, 0);
		float distance = (int)(usData[0]*100.0);
		float result = 0;
		if (distance > 200){
			// true 255, therefore set distance to 255
			result = 200; //clips it at 50
		} else {
			// distance went below 255, therefore reset everything.
			result = distance;
		}
		//lastDistance = distance;
		return result;
	}

}
