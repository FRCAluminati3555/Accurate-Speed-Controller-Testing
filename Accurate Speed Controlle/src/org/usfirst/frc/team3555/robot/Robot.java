package org.usfirst.frc.team3555.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot {
	private AccurateSpeedController speedController;		//regulate the speed of specific talons in sinc    
	private I2C i2c; 										//object which contains the data from the i2c port
	
	private Victor motor1;
	private Victor motor2;
	
	private I2CSensor encoder;
	
    public Robot() {
    	i2c = new I2C(I2C.Port.kOnboard, 0x0A); // 495 should be RPM  (8250 mRPS)
    	encoder = new I2CSensor(i2c, 0x00, 4); 	//data from I2C milli-rev per second
    	speedController = new AccurateSpeedController(encoder, 39844, motor1 = new Victor(0), motor2 = new Victor(1));
    }

    public void operatorControl() {
    	speedController.setSpeed(7000);
        while (isOperatorControl() && isEnabled()) {
        	speedController.update();
        	
//        	encoder.requestData();
//        	total += encoder.getIntValue();
//        	count ++;
//        	
//        	SmartDashboard.putNumber("AVerage Out: ", (float)total / (float)count);
        	Timer.delay(0.05);
        	
//        	SmartDashboard.putNumber("Sample Speed: ", AccurateSpeedController.getTopSpeed(encoder, motor1, motor2));//getEncodeSpeed
//        	Timer.delay(3);
        }
    }
    
    public static class AccurateSpeedController { // REVIEW THIS METHOD
    	private static final int AVERAGE_COUNT = 5;							//takes the average of 5 counts 		
    	private static final int ERROR_TOLERANCE = 1;
    	private static final float ADJUST_RATE = 0.01f;
    	
    	private SpeedController[] controlles;								//all talons 
    	private I2CSensor encoder;											//data from I2C
    	private float wheelRadius; 											// 1/10"
    	
    	private int[] slidingAverageValues;
    	private int currentSampleCount;
    	
    	private float tartgetSpeed; 										// 1/10" per second
    	private float adjust;
    	
    	private float topSpeed;
    	
    	public AccurateSpeedController(I2CSensor encoder, float topSpeed, SpeedController... controlles) {
    		this.controlles = controlles;							
    		this.encoder = encoder;									
    		this.topSpeed = topSpeed;	
    		this.slidingAverageValues = new int[AVERAGE_COUNT];				// Rolling average of array 5 
    		this.currentSampleCount = 0;
    		this.wheelRadius = wheelRadius / 10;							//???
		}
    	
    	public void setSpeed(float inchsPerSecond) {
    		tartgetSpeed = inchsPerSecond * 10;
    		adjust = 0;
    	}
    	
    	public void update() {
    		// Sliding Average
    		encoder.requestData();
    		int encoderValue = encoder.getIntValue(); 						// 1/10" per second
    		if(currentSampleCount < AVERAGE_COUNT)
    			currentSampleCount ++;
    		float average = 0;
    		for(int i = 0; i < AVERAGE_COUNT - 1; i ++) {
    			slidingAverageValues[i] = slidingAverageValues[i + 1];
    			average += slidingAverageValues[i];
    		}
    		
    		slidingAverageValues[AVERAGE_COUNT - 1] = encoderValue;
    		average += slidingAverageValues[AVERAGE_COUNT - 1];
    		average /= (float) currentSampleCount;
    		
    		SmartDashboard.putNumber("Encoder In: ", encoderValue);
    		SmartDashboard.putNumber("Average: ", average);
    		
    		if(currentSampleCount == AVERAGE_COUNT - 1) {
	    		// Error Calculation
	    		if(Math.abs(average - tartgetSpeed) > ERROR_TOLERANCE) {
	    			adjust += (average - tartgetSpeed) * ADJUST_RATE;
	    		}
    		}

    		SmartDashboard.putNumber("Adjust: ", adjust);
    		SmartDashboard.putNumber("TargetSpeed: ", tartgetSpeed);
    		SmartDashboard.putNumber("Adj. Speed: ", (tartgetSpeed + adjust));
    		
    		
    		// Set Speeds
    		double value = (tartgetSpeed + adjust) / topSpeed;
    		SmartDashboard.putNumber("Speed: ", value);
    		for(SpeedController speedController : controlles) {
    			if(tartgetSpeed == 0)
    				speedController.set(0);
    			else
    				speedController.set(value);
    		}
    	}
    	
    	public static double getTopSpeed(I2CSensor encoder, SpeedController... controllers) {
    		for(SpeedController controller : controllers)
    			controller.set(1.0);
    		int total = 0;
    		for(int i = 0; i < 100; i ++) {
    			encoder.requestData();
    			total += encoder.getIntValue();
    			Timer.delay(5.0 / 100.0);
    		}
    		
    		for(SpeedController controller : controllers)
    			controller.set(0);
    		return total / 100.0;
    	}
    	
    	public static double getEncodeSpeed(I2CSensor encoder, SpeedController... controllers) {
    		for(SpeedController controller : controllers)
    			controller.set(1);
    		Timer.delay(0.5);
    		
    		int index = 0; double average = 100;
    		int[] slidingValue = new int[10];
    		for(; index < slidingValue.length; index ++) {
    			encoder.requestData();
    			slidingValue[index] = encoder.getIntValue();
    			Timer.delay(0.1);
    		} 
    		
    		while(average > 1) {
    			for(SpeedController controller : controllers)
        			controller.set(controller.get() - 0.001);
    			
    			encoder.requestData();
    			slidingValue[index - 1] = encoder.getIntValue();
    			
    			double total = 0;
    			for(int i = 0; i < index - 1; i ++) {
    				total += slidingValue[i];
    				slidingValue[i] = slidingValue[i + 1];
    			} total += slidingValue[slidingValue.length - 1];
    			
    			average = total / (float)index;
    			SmartDashboard.putNumber("Average: ", average);
    			Timer.delay(0.1);
    		}
    		
    		while(average < 1.1) {
    			for(SpeedController controller : controllers)
        			controller.set(controller.get() + 0.001);
    			
    			encoder.requestData();
    			slidingValue[index - 1] = encoder.getIntValue();
    			
    			double total = 0;
    			for(int i = 0; i < index - 1; i ++) {
    				total += slidingValue[i];
    				slidingValue[i] = slidingValue[i + 1];
    			} total += slidingValue[slidingValue.length - 1];
    			
    			average = total / (float)index;
    			SmartDashboard.putNumber("Average: ", average);
    			Timer.delay(0.1);
    		}
    		
    		double result = 0;
    		for(SpeedController controller : controllers)
    			result += controller.get();
    		result /= (float) controllers.length;
    		
    		for(SpeedController controller : controllers)
    			controller.set(0);
    		
    		return result;
    	}
    }
    
    public static class I2CSensor {
    	private final I2C i2c;
    	private int requestAddress;
    	private byte[] readArray;
    	
    	public I2CSensor(I2C i2c, int requestAddress, int arraySize) {
    		if(arraySize > 7 || arraySize < 1) throw new IllegalArgumentException("arraySize must be < 8 || > 0");
    		this.i2c = i2c;
    		
    		this.requestAddress = requestAddress;
    		this.readArray = new byte[arraySize];
    	}
    	
    	public void requestData() {
    		i2c.write(0, requestAddress);
    		Timer.delay(0.01);
    		i2c.readOnly(readArray, readArray.length);
    	}
    	
		public char getCharValue() {
			if(readArray.length < 1) {
				throw new IllegalStateException("Can not create Char with only " + readArray.length + " bytes."
						+ " At least 1 byte is needed");
			}
			
			return (char) readArray[0];
		}
    	
    	public byte getByteValue() {
			if(readArray.length < 1) {
				throw new IllegalStateException("Can not create Byte with only " + readArray.length + " bytes."
						+ " At least 1 byte is needed");
			}
			
			return readArray[0];
    	}
    	
		public short getShortValue() {
			if(readArray.length < 2) {
				throw new IllegalStateException("Can not create Short with only " + readArray.length + " bytes."
						+ " At least 2 byte is needed");
			}
			
			return (short) (((short)(readArray[0] << 8) & 0xFF00) | ((short)readArray[1] & 0x00FF));
    	}
		
		public int getIntValue() {
			if(readArray.length < 4) {
				throw new IllegalStateException("Can not create Int with only " + readArray.length + " bytes."
						+ " At least 4 byte is needed");
			}
			
			return (readArray[0] << 24 & 0xFF000000) | (readArray[1] << 16 & 0x00FF0000)
					| (readArray[2] << 8 & 0x0000FF00) | (readArray[3] & 0x000000FF);
		}
		
		public long getLongValue() {
			if(readArray.length < 8) {
				throw new IllegalStateException("Can not create Long with only " + readArray.length + " bytes."
						+ " At least 8 byte is needed");
			}
			
			return 	((long) (
					(readArray[0] << 24 & 0xFF000000) | (readArray[1] << 16 & 0x00FF0000) |
					(readArray[2] << 8  & 0x0000FF00) | (readArray[3] 	    & 0x000000FF)) << 32) 
					
					|
					
					((long) (
					(readArray[4] << 24 & 0xFF000000) | (readArray[5] << 16 & 0x00FF0000) |
					(readArray[6] << 8  & 0x0000FF00) | (readArray[7] 	    & 0x000000FF)) & 0xFFFFFFFF);
		}
		
		public float getFloatValue() {
			throw new IllegalStateException("This system does not suppot floting point numbers");
		}
		
		public double getDoubleValue() {
			throw new IllegalStateException("This system does not suppot floting point numbers");
		}
		
		public boolean getBooleanValue() {
			if(readArray.length < 1) {
				throw new IllegalStateException("Can not create Boolean with only " + readArray.length + " bytes."
						+ " At least 1 byte is needed");
			}
			
			return readArray[0] != 0;
		}
    }
}
