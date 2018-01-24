package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

//	Credit to Team 5188 Classified Robotics for the source code

public class M_I2C {
	//	Connects the SDA on the I2C port on the RoboRIO to address 20 on the Arduino
	private static I2C Wire = new I2C(Port.kOnboard, 20);

	//	Sets max number of bytes for reading from Arduino
	private static final int MAX_BYTES = 32;

	//	A method for writing data to the Arduino through the I2C
	public void write(String input){ 

		//	Creates a char array from the input string
		char[] CharArray = input.toCharArray();

		//	Creates a byte array that is the same length as the char array
		byte[] WriteData = new byte[CharArray.length];

		//	Writes the char array into the byte array
		for (int i = 0; i < CharArray.length; i++) {
			WriteData[i] = (byte) CharArray[i];
		}

		//	Writes the byte array through the I2C onto the Arduino
		Wire.transaction(WriteData, WriteData.length, null, 0);



	}


	//	A method for saving the data from the Arduino to a PixyPacket
	public PixyPacket getPixy(){

		//	Splits the string every time a "|" is seen in order to separate x, y, and area
		String info[] = read().split("\\|");

		//	Creates a new packet to store the data from the Arduino
		PixyPacket pkt = new PixyPacket();   

		//	Sets values to negative one so we will know there is no data
		if(info[0].equals("none") || info[0].equals("")){ 

			//  Debug prints to see whether the PixyCam sees no data, or if the Arduino is giving no data|
			if(info[0].equals("none")){
				System.out.println("The pixy detects no blocks");
			}
			else if(info[0].equals("")){
				System.out.println("No data from arduino");
			}

			//	These values cannot be -1 naturally, so it acts as a flag
			pkt.x = -1;
			pkt.y = -1;
			pkt.area = -1;
		}
		//	Checks to make sure the correct amount of data is coming from the Arduino
		else if(info.length == 3){

			//	Stores data from Arduino in pkt
			pkt.x = Double.parseDouble(info[0]);
			pkt.y = Double.parseDouble(info[1]);
			pkt.area = Double.parseDouble(info[2]);

		}
		//	Prints an error if there is an incorrect amount of data
		else{
			System.out.println("ERROR: Incorrect amount of data read");
		}

		//	Outputs the packet for later use
		return pkt;

	}

	// 	NOTE: This method is written very poorly and needs to be rewritten (especially the ternary)
	//	Method to read the data from Arduino, called by the method getPixy()
	private String read(){
		//	Creates a byte array to store data from the Arduino
		byte[] data = new byte[MAX_BYTES];

		// Reads data off of Arduino and stores it in data
		Wire.read(20, MAX_BYTES, data);

		// Creates a string from the byte array
		String output = new String(data);

		// Creates an integer equal to the length of the string
		int pt = output.indexOf((char)255);

		//	Returns the string received from the Arduino
		return (String) output.subSequence(0, pt < 0 ? 0 : pt);

	}





}