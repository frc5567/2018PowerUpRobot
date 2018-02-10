package org.usfirst.frc.team5567.robot;

public class averagePosition {
	int index;
	double valueArray[];
	PixyPacket pkt;
	M_I2C i2c;
	double averageValue;
	double previousAverageValue;


	public averagePosition(){
		index = 0;
		valueArray = new double[3];
		i2c = new M_I2C();
		pkt = i2c.getPixy();
		averageValue = 0;

	}

	public void printAverageValues(){
		
		//	Refreshes Pixy Values
		pkt = i2c.getPixy();
		
		//	Assigns Pixy values to an array
		if(index < 3){
			valueArray[index++] = pkt.x;
		}
		else if(index == 3){
			
			//	Adds up the values for averaging
			for(int i = 0; i < 3; i++){
				averageValue += valueArray[i];
			}
			
			//	Gets the average value
			averageValue /= 3;
			
			//	If this is the first time through, it won't compare it to a previous value
			if(previousAverageValue == -1){
				previousAverageValue = averageValue;
				System.out.println(averageValue);
			}
			
			//	It won't change the previous value or print the new value if there is an outlier
			else if(averageValue < previousAverageValue - 0.1 || averageValue > previousAverageValue + 0.1){
				System.out.println("Outlier Exception");
			}
			
			//	Prints the average value and assigns the current value to the previous value
			else{
				System.out.println(averageValue);
				previousAverageValue = averageValue;
			}
			
			index = 0;
		} 

	}
}
