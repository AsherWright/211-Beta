import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
/**
 * @author danielebercovici
 * @version 1.0
 */
public class BlockDetectorOld extends Thread {
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
     * @param colorSensor sample provider for colored sensor
     * @param colorData sample data for colored sensor
     * @param navi Navigator class Object
     * @param odo Odometer class Object
     * @param leftMotor EV3 Motor on the left
     * @param rightMotor EV3 Motor on the right
     * @param UltrasonicPoller frontPollerusSensor, sample provider for ultrasonic sensor, usData sample data for Ultrasonic sensor
     * @param verticalArmMotor EV3 Motor for up/down movement of arm
     * @param horizontalArmMotor EV3 Motor for open/close movement of arm
     */
    public BlockDetectorOld(ColorSensorPoller blockPoller, Navigation navi, Odometer odo, EV3LargeRegulatedMotor leftMotor,
                         EV3LargeRegulatedMotor rightMotor,UltrasonicPoller frontPoller, EV3LargeRegulatedMotor verticalArmMotor, EV3LargeRegulatedMotor horizontalArmMotor) {
        //get incoming values for variables
        this.blockPoller = blockPoller;
        this.odo = odo;
        this.navi= navi;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
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
     * Method that Checks if Block is the right color, set global variable isReadingBlock true if right block and false otherwise.
     */
    public void run(){
        blockPoller.setMode(2);
        investigateBlock();
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

        lTheta = pos[2]-odo.getAng();
        
        navi.turnTo(pos[2], false);
   
        
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
  
        rTheta = pos[2]-odo.getAng();
        
        navi.turnTo(pos[2], true);
        
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
        
       //for testing
        System.out.println("difference:"+difference);
        
        if(difference <= 88 && difference >= 65)//within center range
        {
            
            USDistance = getFilteredUSData();
            while(USDistance > DETECTIONRANGE) //get within light sensor range
            {
                rightMotor.forward();
                leftMotor.forward();
                USDistance = getFilteredUSData();
            }
            rightMotor.stop(true);
            leftMotor.stop(true);
            
            //get within light sensor range
            rightMotor.rotate(convertDistance(WHEEL_RADIUS,5), true);
            leftMotor.rotate(convertDistance(WHEEL_RADIUS,5), false);
            
            isFlagDetected();
        }
        
        else //too far to right or left 
        {
            
            //drive into the block to straighten it out
            rightMotor.rotate(convertDistance(WHEEL_RADIUS,17),true); 
            leftMotor.rotate(convertDistance(WHEEL_RADIUS,17),false);
            
//            rightMotor.rotate(convertDistance(WHEEL_RADIUS,-DETECTIONRANGE),true);
//            leftMotor.rotate(convertDistance(WHEEL_RADIUS,-DETECTIONRANGE),false);
      
            isFlagDetected();
        }
        
    }
    
    /**
     * Method determines whether to capture flag by checking value of isReadingBlock calling pickUp method is true or returning robot to initial position otherwise
     */
    public void isFlagDetected()
    {
//        //get within light sensor range
//        rightMotor.rotate(convertDistance(WHEEL_RADIUS,3), true);
//        leftMotor.rotate(convertDistance(WHEEL_RADIUS,3), false);
        boolean isFlag = isFlag();
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
            navi.turnTo(pos[2], true);
           
        }
    }
    
    /**
     * Method picks up flag its facing
     */
    public void pickUp()
    {
        //back up
//        while(USDistance <= 15)//arms dont hit at 21cm. perfect dist: 13cm
//        {
//            rightMotor.backward();
//            leftMotor.backward();
//            USDistance = getFilteredUSData();
//        }
//        rightMotor.stop(false);
//        leftMotor.stop(false);
    	rightMotor.rotate(convertDistance(WHEEL_RADIUS,-13), true);
    	leftMotor.rotate(convertDistance(WHEEL_RADIUS,-13), false);
        
        
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
        rightMotor.setSpeed(100);
        leftMotor.setSpeed(100);
        rightMotor.rotate(convertDistance(WHEEL_RADIUS,-10),true);//arms dont hit at 15cm, perf dist: -8
        leftMotor.rotate(convertDistance(WHEEL_RADIUS,-10),false);
        
        
        //close arms
        horizontalArmMotor.rotate(-130, false);
        Sound.beep();
        
        //lift up arms
        verticalArmMotor.rotate(180,false);
    }
    public boolean isFlag()
    {
        boolean isFlag = false;
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
    
}
