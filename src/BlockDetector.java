import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
/**
 * @author danielebercovici
 * @version 1.1
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
    private static final double BANDWIDTH = 16.2;
    private final int DETECTIONRANGE = 4;
    //ultrasonic sensor variables
    private double USDistance;
    private UltrasonicPoller frontPoller;
    //Reading properties
    private String blockType;
    private boolean isFlag; //indicates if flag was found
    //odometer and navigator
    Odometer odo;
    Navigation navi;
    //position variable, [x, y, theta]
    private double[] pos = new double [3];
    //The motors for moving the robot and for moving the arms to capture the flag
    EV3LargeRegulatedMotor leftMotor;
    EV3LargeRegulatedMotor rightMotor;
    EV3LargeRegulatedMotor verticalArmMotor;
    EV3LargeRegulatedMotor horizontalArmMotor;
    ColorSensorPoller blockPoller;
    
    /**
     * Class constructor
     * @param colorSensorPoller sample and data provider for colored sensor
     * @param navi Navigator class Object
     * @param odo Odometer class Object
     * @param leftMotor EV3 Motor on the left
     * @param rightMotor EV3 Motor on the right
     * @param UltrasonicPoller frontPollerusSensor, sample provider for ultrasonic sensor, usData sample data for Ultrasonic sensor
     * @param verticalArmMotor EV3 Motor for up/down movement of arm
     * @param horizontalArmMotor EV3 Motor for open/close movement of arm
     */
    public BlockDetector(ColorSensorPoller blockPoller, Navigation navi, Odometer odo,UltrasonicPoller frontPoller, EV3LargeRegulatedMotor verticalArmMotor, EV3LargeRegulatedMotor horizontalArmMotor) {
        //get incoming values for variables
        this.blockPoller = blockPoller;
        this.odo = odo;
        this.navi= navi;
		EV3LargeRegulatedMotor[] motors = odo.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];
        this.frontPoller = frontPoller;
        this.verticalArmMotor = verticalArmMotor;
        this.horizontalArmMotor = horizontalArmMotor;
        
        //initialize variables
        blockType = "";
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
     * Method runs Block Detection setting light sensor mode to color ID
     */
    public void run(){
        blockPoller.setMode(2);
       // investigateBlock();
    }
    /**
     * Method that gets close to block in order to get accurate readings for the light sensor, calls isFlagDetected method
     */
    public void investigateBlock()
    {
        //record initial position
        odo.getPosition(pos);
        
        double lTheta; //falling edge angle
        double rTheta; //rising edge angle
        
        
        USDistance = getFilteredUSData();
        
        //turn left till cant see block, record angle
        while(USDistance <= 15)
        {
            USDistance = getFilteredUSData();
            leftMotor.setSpeed(50);
            rightMotor.setSpeed(50);
            rightMotor.forward();
            leftMotor.backward();
        }
        rightMotor.stop(true);
        leftMotor.stop(true);
        Sound.beep();
        lTheta = pos[2]-odo.getAng();
        
        navi.turnTo(pos[2], false);
        Sound.beep();
        
        USDistance = getFilteredUSData();
        
        //turn right till cant see block, record angle
        while(USDistance <= 15)
        {
            USDistance = getFilteredUSData();
            leftMotor.setSpeed(50);
            rightMotor.setSpeed(50);
            leftMotor.forward();
            rightMotor.backward();
        }
        leftMotor.stop(true);
        rightMotor.stop(true);
        Sound.beep();
        rTheta = pos[2]-odo.getAng();
        
        navi.turnTo(pos[2], true);
        
        Sound.beep();
        Sound.beep();
        rightMotor.setSpeed(100);
        leftMotor.setSpeed(100);
        USDistance = getFilteredUSData();
        
        /*calculate position in relation to the center of block
         * 1: within center threshold, drive within DETECTIONRANGE detect if its good a) good: pickup b) bad: go back to original position
         * 2: too far to right or left, hit block, back up to DETECTIONRANGE, detect if it is good a) good: pickup b) bad: go back to original position
         */
        double difference = Math.abs(rTheta - lTheta);
        
        if (difference > 180 ) //take orientation of grid into account
        {
            difference = 360-difference;
        }
        
        if(difference <= 88 && difference >= 65)//within center of block
        {
            Sound.buzz();
            Sound.buzz();
            
            
            while(getFilteredUSData() > DETECTIONRANGE) //get within 4cm
            {
                rightMotor.forward();
                leftMotor.forward();
            }
            rightMotor.stop(true);
            leftMotor.stop(true);
            
            //get within light sensor range (cant use ultrasonic because only detects till 4cm)
            rightMotor.rotate(convertDistance(WHEEL_RADIUS,5), true);
            leftMotor.rotate(convertDistance(WHEEL_RADIUS,5), false);
            
            isFlagDetected();
        }
        
        else //block at angle 
        {
            Sound.buzz();
            while(getFilteredUSData() > DETECTIONRANGE) //get within 4cm
            {
                rightMotor.forward();
                leftMotor.forward();
     
            }
            rightMotor.stop(true);
            leftMotor.stop(true);
            //drive into the block to straighten it out
            rightMotor.rotate(convertDistance(WHEEL_RADIUS,7),true); 
            leftMotor.rotate(convertDistance(WHEEL_RADIUS,7),false);
      
            isFlagDetected();
        }
        
    }
    
    /**
     * Method determines whether to capture flag by checking value of isFlag calling pickUp method is true or returning robot to initial position otherwise
     */
    public void isFlagDetected()
    {
    	//check if flag or just other block
        isFlag = investigateFlag();
        rightMotor.rotate(convertDistance(WHEEL_RADIUS,-3), true);
        leftMotor.rotate(convertDistance(WHEEL_RADIUS,-3), false);
        
        if(isFlag) //capture flag
        {      
            pickUp();
        }
        else //not flag, return to initial position
        {
        	rightMotor.setSpeed(100);
        	leftMotor.setSpeed(100);
        	//back up a bit 
        	rightMotor.rotate(convertDistance(WHEEL_RADIUS,-5), true);
            leftMotor.rotate(convertDistance(WHEEL_RADIUS,-5), false);
            navi.travelTo(pos[0], pos[1]);
            //navi.turnTo(pos[2], true);
            Sound.beep();
        }
    }
    
    /**
     * Method picks up flag its facing
     */
    public void pickUp()
    {
        //back up
    	rightMotor.rotate(-convertDistance(WHEEL_RADIUS,13), true);
    	leftMotor.rotate(-convertDistance(WHEEL_RADIUS,13), false);
          
        //turn around
        rightMotor.rotate(convertAngle(WHEEL_RADIUS,BANDWIDTH,165),true);
        leftMotor.rotate(convertAngle(WHEEL_RADIUS,BANDWIDTH,-165),false);
        
        //open arm
        horizontalArmMotor.setSpeed(100);
        horizontalArmMotor.rotate(100, false);
        
        //lower arms
        verticalArmMotor.setSpeed(100);
        verticalArmMotor.rotate(-180);
        try {
            Thread.sleep(2000);
        }
        catch (Exception e) {
        }
        
        //back up
        rightMotor.setSpeed(30);
        leftMotor.setSpeed(30);
        rightMotor.rotate(-convertDistance(WHEEL_RADIUS,10),true);//arms dont hit at 15cm, perf dist: -8
        leftMotor.rotate(-convertDistance(WHEEL_RADIUS,10),false);
            
        //close arms
        horizontalArmMotor.rotate(-130, false);
        Sound.beep();
        
        //lift up arms
        verticalArmMotor.rotate(180,false);
        leftMotor.setSpeed(100);
        rightMotor.setSpeed(100);
        
    }
    /**
     * This method uses color sensor to identify if block is color of flag
     * @return boolean value, true if flag color, false otherwise
     */
    public boolean investigateFlag()
    {
        isFlag = false;
        //gets the data from the color sensor.
        float colorData[] = blockPoller.getColorData();
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
            isFlag = false;
        }else if(BlueBlockError[0] < DETECTIONTHRESHOLDERROR && BlueBlockError[1] < DETECTIONTHRESHOLDERROR &&  BlueBlockError[2] < DETECTIONTHRESHOLDERROR ){
            blockType = "BLOCK";
            isFlag =true;
        }else if(DarkBlueBlockError[0] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[1] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[2] < DETECTIONTHRESHOLDERROR){
            blockType = "BLOCK";
            isFlag = true;
        }else{
            blockType = "NOT BLOCK";
            isFlag = false;
        }

        return isFlag;
    }
    
    /**
     * This method converts a distance to an angle in degrees
     * @param radius the wheel radius
     * @param distance desired distance to convert
     * @return angle in degrees
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
    
    /**
     * This method converts an angle to distance in cm
     * @param radius the wheel radius
     * @param width the bandwidth of the two wheels 
     * @param angle desired angle to convert
     * @return result of method convertDistanc() distance in cm
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    /**
     * This method filters the Ultrasonic Data. clips data if too far
     * @return result
     */
    private float getFilteredUSData() {
        float distance = frontPoller.getUsData();
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

    public String getBlockType(){
        synchronized (this) {
            return blockType;	
        }
        
    }
    public Boolean isFlag(){
        synchronized (this) {
            return isFlag;	
        }
    }
}
