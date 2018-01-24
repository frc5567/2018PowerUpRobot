package org.usfirst.frc.team5567.robot;

import org.usfirst.frc.team5567.robot.PixyPacket;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//	Credit to Team 5188 Classified Robotics for the source code

public class PixyExample {

	//	Sets up the I2C interface
	M_I2C i2c = new M_I2C();

	//	Creates a Pixy packet to transfer Pixy data
	PixyPacket pkt = i2c.getPixy();

	/*
	 * 	This method will turn the robot to face the object the PixyCam has locked on to and drive towards it
	 * 	The Drive Train code is commented out for testing purposes
	 */
	public void centerOnObject(DifferentialDrive driveTrain){

		System.out.println("Entered center on object");
		pkt = i2c.getPixy();

		//	If data exists, proceed (-1 is the default value)
		if(pkt.x != -1){

			System.out.println("Data Exists");

			//	While the object is not center
			if(pkt.x < .48 || pkt.x > .52){
				System.out.println("Object is not in center");

				//	If the Pixy sees the object on the left side of robot, turn left
				if(pkt.x < .48){
					System.out.println("Object on left");
					//driveTrain.arcadeDrive(0, -0.2);
					Timer.delay(0.05);
				}

				//	If the Pixy sees the object on the right side of robot, turn right
				else if(pkt.x > .52){
					System.out.println("Object on right");
					//driveTrain.arcadeDrive(0, 0.2);
					Timer.delay(0.05);
				}

				//	Refresh the data
				pkt = i2c.getPixy();

				//	Print the data for reference
				System.out.println("XPos: " + pkt.x + "  YPos:  " + pkt.y);

			}

			//	If the object is in the center
			else if(pkt.x > .48 || pkt.x < .52){
				System.out.println("Object is in center");
				//	Drives forward
				//driveTrain.arcadeDrive(0.3,0);
				Timer.delay(0.05);

				System.out.println("XPos:  " + pkt.x + "  YPos:   " + pkt.y);

			}
		}
		else if(pkt.x == -1){
			System.out.println("Data does not exist");
		}
	}

}
