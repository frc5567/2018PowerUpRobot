package org.usfirst.frc.team5567.robot;

public class ComplementaryFilter {

//  Constants for the filter
	public static final float THRESHOLD = (float) 0.075;
	public float FILTER_COEFFICIENT = 0;
	public static final float DELTA_TIME = (float) 0.01;
	public float filteredAngle = (float) 0.0;
	
	//	The constructor. Does nothing but create an instance, really.
	public ComplementaryFilter(){
		
	}
	
	/**
	 * This is the method that will filter the data from the gyro and the accelerometer
	 * and return the filtered value. The filtered value is a float.
	 * 
	 * @param gyroData The raw gyro angle to be used in the filter.
	 * @param accData The raw accelerometer data to be used in the filter.
	 * 
	 * @return filteredAngle The float value returned by the method.
	 */  
	public float FilterAngle(float gyroData, float accData){
		
		//	Calculates the filter coefficient for use in the filter equation
		FILTER_COEFFICIENT = THRESHOLD/(THRESHOLD + DELTA_TIME);
		
		//	The main filter equation
		filteredAngle = FILTER_COEFFICIENT * (filteredAngle + gyroData * DELTA_TIME) + (1-FILTER_COEFFICIENT) * accData;
		
		return filteredAngle;
		
	}
	
	
}
