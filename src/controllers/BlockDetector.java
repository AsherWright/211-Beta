package controllers;
import odometry.Odometer;
import pollers.ColorSensorPoller;
import pollers.UltrasonicPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * @author danielebercovici
 * @version 2.0
 */
public class BlockDetector extends Thread {
    //constants
    //class variables
    //these are the different block profiles. They are initialized in the constructor
    private double[] blueBlockReading;
    private double[] darkBlueBlockReading;
    private double[] redBlockReading;
    private double[] yellowBlockReading;
    private double[] whiteBlockReading;
    
    //the error that R,G, B can be off for it to still consider it a certain object.
    private static final double DETECTIONTHRESHOLDERROR = 0.5;
    private static final double WHEEL_RADIUS = 2.1;
    private static final double BANDWIDTH = 16.2;
    private final int DETECTIONRANGE = 4;
    //ultrasonic sensor variables
    private UltrasonicPoller frontPoller;
    //Reading properties
    private String blockType;
    private boolean isFlag; //indicates if flag was found
    private int flagType;
    private double lTheta; //falling edge angle
    private double rTheta; //rising edge angle
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
     * Class Constructor
     * @param blockPoller Color Sensor to identify the colors of the block
     * @param navi Navigator class Object
     * @param odo Odometer class Object
     * @param frontPoller USsensor on the front of robot
     * @param verticalArmMotor EV3 Motor for up/down movement of arm
     * @param horizontalArmMotor EV3 Motor for open/close movement of arm
     * @param flagType wifi flag color to find
     */
    public BlockDetector(ColorSensorPoller blockPoller, Navigation navi, Odometer odo,UltrasonicPoller frontPoller, EV3LargeRegulatedMotor verticalArmMotor, EV3LargeRegulatedMotor horizontalArmMotor, int flagType) {
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
        redBlockReading = new double[3];
        yellowBlockReading = new double[3];
        whiteBlockReading = new double[3];
        blueBlockReading[0] = 0.95;
        blueBlockReading[1] = 1.4;
        blueBlockReading[2] = 1.15;
        darkBlueBlockReading[0] = 0.2;
        darkBlueBlockReading[1] = 0.5;
        darkBlueBlockReading[2] = 0.7;   

        redBlockReading[0] = 0.9;
        redBlockReading[1] = 0.15;
        redBlockReading[2] = 0.13;  
        yellowBlockReading[0] = 1.5;
        yellowBlockReading[1] = 1.1;
        yellowBlockReading[2] = 0.1;
        whiteBlockReading[0] = 2.0;
        whiteBlockReading[1] = 2.2;
        whiteBlockReading[2] = 1.3;
        this.flagType = flagType;

    }
    
    /**
     * Method runs Block Detection setting light sensor mode to color ID
     */
    public void run(){
        blockPoller.setMode(2);
////        while(true){
////        investigateFlag();
////        try {
////			Thread.sleep(50);
////		} catch (InterruptedException e) {
////			// TODO Auto-generated catch block
////			e.printStackTrace();
////		}
//        }
    }
    /**
     * Calculates the path to get close enough to block in order to get accurate readings from the light sensor, calls isFlagDetected method
     */
    public void investigateBlock()
    {
        //initialize variables
        odo.getPosition(pos);
        int generalSpeed = 100;
        double angleTraveled = odo.getAng();
        boolean goneHalf = false;
        
        //spin until we see the block with front sensor
        while(getFilteredUSData() > 29 && Math.abs(odo.getAng()-angleTraveled) < 60){
        	leftMotor.setSpeed(generalSpeed);
        	rightMotor.setSpeed(generalSpeed);
        	leftMotor.backward();
        	rightMotor.forward();
        	if(Math.abs(odo.getAng()-pos[2]) > 100){
        		goneHalf = true;
        	}
        	if(goneHalf && Math.abs(odo.getAng()-pos[2]) < 5){
        		return;
        	}
        }
    	try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	
        //turn left till cant see block, record angle
        while(getFilteredUSData() <= 33)
        {
            leftMotor.setSpeed(generalSpeed);
            rightMotor.setSpeed(generalSpeed);
            rightMotor.forward();
            leftMotor.backward();
        }
        rightMotor.stop(true);
        leftMotor.stop(true);
       angleTraveled = odo.getAng();
        lTheta = odo.getAng();

        while(getFilteredUSData() >= 29)
        {
            leftMotor.setSpeed(generalSpeed);
            rightMotor.setSpeed(generalSpeed);
            rightMotor.backward();
            leftMotor.forward();
        }
    	try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	
        
        //turn right till cant see block, record angle
        while(getFilteredUSData() <= 29)
        {		
        	boolean doIt = true; //clip angle to avoid detecting two blocks as one
        	if(odo.getAng() > 180 && angleTraveled > 180 || odo.getAng() < 180 && angleTraveled < 180){
        		 if(Math.abs(odo.getAng() - angleTraveled) > 73){
        			 doIt = false;
        		 }
        	}else if(odo.getAng() < 180 && angleTraveled > 180){
        		if(Math.abs(odo.getAng() - (360-angleTraveled)) > 73){
        			doIt = false;
        		}
        	}
        	
        	if(doIt){
        		leftMotor.setSpeed(generalSpeed);
        		rightMotor.setSpeed(generalSpeed);
        		leftMotor.forward();
        		rightMotor.backward();        		
        	}else{
        		break;
        	}
        }
        leftMotor.stop(true);
        rightMotor.stop(true);
        rTheta = odo.getAng();
        

        
        /*calculate path angle in relation to the position of the block and the position of the robot
         *  a) good: pickup b) bad: go back to original position
         */
        double newTheta;
        if (rTheta < lTheta)
        {
        	newTheta = (rTheta+lTheta)/2;
        }else{
        	newTheta = (rTheta-360+lTheta)/2;
        	if(newTheta < 0){
        		newTheta +=360;
        	}
        }
        
//        if(Math.abs(newTheta-pos[2]) > 45){
//        	return;
//        }
        
        navi.turnTo(newTheta, true);
        Sound.buzz();
        while(getFilteredUSData() > DETECTIONRANGE) //get within 4cm
          {
        	rightMotor.setSpeed(100);
        	leftMotor.setSpeed(100);
              rightMotor.forward();
              leftMotor.forward();
          }
          rightMotor.stop(true);
          leftMotor.stop(true);
          
          //get within light sensor range (cant use ultrasonic because only detects till 4cm)
          rightMotor.rotate(convertDistance(WHEEL_RADIUS,6), true);
          leftMotor.rotate(convertDistance(WHEEL_RADIUS,6), false);
        
          isFlagDetected();
   }

    
    /**
     * Determines whether to capture flag by checking value of isFlag calling pickUp method is true or returning robot to initial position otherwise
     */
    private void isFlagDetected()
    {
    	//check if flag or just other block
        isFlag = investigateFlag();
        rightMotor.rotate(convertDistance(WHEEL_RADIUS,-3), true);
        leftMotor.rotate(convertDistance(WHEEL_RADIUS,-3), false);

        
        if(isFlag) //capture flag
        {      
            pickUp();
            Sound.beepSequenceUp();
        }
        else //not flag 
        {
        	Sound.beepSequence();
        	rightMotor.setSpeed(130);
        	leftMotor.setSpeed(130);
        
        	//back up a bit 
        	rightMotor.rotate(convertDistance(WHEEL_RADIUS,-5), true);
            leftMotor.rotate(convertDistance(WHEEL_RADIUS,-5), false);
            //return to initial position
            navi.travelTo(pos[0], pos[1]);
        }
    }
    
    /**
     * Picks up flag its facing
     */
    private void pickUp()
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
        rightMotor.setSpeed(70);
        leftMotor.setSpeed(70);
        rightMotor.rotate(-convertDistance(WHEEL_RADIUS,10),true);
        leftMotor.rotate(-convertDistance(WHEEL_RADIUS,10),false);
            
        //close arms
        horizontalArmMotor.rotate(-130, false);
        
        //lift up arms
        verticalArmMotor.rotate(180,false);
        leftMotor.setSpeed(100);
        rightMotor.setSpeed(100);
        
    }
    /**
     * Uses color sensor to identify if block is color of flag
     * @return boolean value, true if flag color, false otherwise
     */
    private boolean investigateFlag()
    {
        isFlag = false;
        //gets the data from the color sensor.
        float colorData[] = blockPoller.getColorData();
        //checks the reading and compares it to each profile.
        double[] BlueBlockError = new double[3];
        double totalNoObjectError = 0;
        double[] DarkBlueBlockError  = new double[3];
        double[] redBlockError = new double[3];
        double[] yellowBlockError = new double[3];
        double[] whiteBlockError = new double[3];
        
        //go through R,G,B
        for(int i = 0; i < 3; i++){
            BlueBlockError[i] = Math.abs(colorData[i]*10 - blueBlockReading[i]);
            totalNoObjectError += Math.abs(colorData[i]*10);
            DarkBlueBlockError[i] = Math.abs(colorData[i]*10 - darkBlueBlockReading[i]);
            redBlockError[i] = Math.abs(colorData[i]*10-redBlockReading[i]);
            whiteBlockError[i] = Math.abs(colorData[i]*10-whiteBlockReading[i]);
            yellowBlockError[i] = Math.abs(colorData[i]*10-yellowBlockReading[i]);
            
        }
        //If our reading is within the allowed number to consider it a blue block, update what it sees.
        if(totalNoObjectError < 0.25){
            blockType = "";
            isFlag = false;
        }else if(whiteBlockError[0] < DETECTIONTHRESHOLDERROR && whiteBlockError[1] < DETECTIONTHRESHOLDERROR && whiteBlockError[2] < DETECTIONTHRESHOLDERROR){
            blockType = "WHITE BLOCK";
            if(flagType == 4){
                isFlag = true;
            }else{
            	isFlag = false;
            }
        }else if(BlueBlockError[0] < DETECTIONTHRESHOLDERROR && BlueBlockError[1] < DETECTIONTHRESHOLDERROR &&  BlueBlockError[2] < DETECTIONTHRESHOLDERROR ){
            blockType = "BLUE BLOCK";
            if(flagType == 1){
                isFlag = true;
            }else{
            	isFlag = false;
            }
        }else if(DarkBlueBlockError[0] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[1] < DETECTIONTHRESHOLDERROR && DarkBlueBlockError[2] < DETECTIONTHRESHOLDERROR){
            blockType = "DARK BLUE BLOCK";
            if(flagType == 5){
                isFlag = true;
            }else{
            	isFlag = false;
            }
        }else if(redBlockError[0] < DETECTIONTHRESHOLDERROR && redBlockError[1] < DETECTIONTHRESHOLDERROR && redBlockError[2] < DETECTIONTHRESHOLDERROR){
            blockType = "RED BLOCK";
            if(flagType == 2){
                isFlag = true;
            }else{
            	isFlag = false;
            }
        
        }else if(yellowBlockError[0] < DETECTIONTHRESHOLDERROR && yellowBlockError[1] < DETECTIONTHRESHOLDERROR && yellowBlockError[2] < DETECTIONTHRESHOLDERROR){
            blockType = "YELLOW BLOCK";
            if(flagType == 3){
                isFlag = true;
            }else{
            	isFlag = false;
            }
        }

        return isFlag;
    }
    
    /**
     * Converts a distance to an angle in degrees
     * @param radius the wheel radius
     * @param distance desired distance to convert
     * @return angle in degrees
     */
    private static int convertDistance(double radius, double distance) {
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }
    
    /**
     * Converts an angle to distance in cm
     * @param radius the wheel radius
     * @param width the bandwidth of the two wheels 
     * @param angle desired angle to convert
     * @return result of method convertDistanc() distance in cm
     */
    private static int convertAngle(double radius, double width, double angle) {
        return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    /**
     * Filters the Ultrasonic Data. clips data if too far
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
