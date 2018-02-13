package org.usfirst.frc.team5567.robot;

import org.usfirst.frc.team5567.robot.PixyPacket;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//	Credit to Team 5188 Classified Robotics for the source code

public class PixyCrate {

	//  Variables for movement
	public static final double STOP_AREA = 0.1;
	public static final double CENTER_POSITION = 0.5;
	public static final double CENTER_THRESHOLD = 0.075;
	public static final double ROTATION_SPEED = 0.15;
	public static final double DRIVE_SPEED = 0.2;
	
	double maxArea = 0;
	
	//	Used to determine if it is the first time through so it avoids a NullPointerException
	boolean firstPixyFlag = true;
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

		//	Checks if first time through
		if(firstPixyFlag){
			firstPixyFlag = false;
			maxArea = pkt.area;
		}
		//	Sets maxArea so we have a stable value to evaluate
		else if(maxArea < pkt.area){
			maxArea = pkt.area;
		}
		//	Stops the robot if the target is too close
		if(maxArea > STOP_AREA){
			driveTrain.arcadeDrive(0, 0, false);
			return;
		}
		System.out.println(maxArea);



		//	If data exists, proceed (-1 represents the absence of data)
		if(pkt.x != -1){

			System.out.println("Data Exists");

			//	While the object is not center
			if(pkt.x < CENTER_POSITION - CENTER_THRESHOLD || pkt.x > CENTER_POSITION + CENTER_THRESHOLD){
				System.out.println("Object is not in center");

				//	If the Pixy sees the object on the left side of robot, turn left
				if(pkt.x < CENTER_POSITION - CENTER_THRESHOLD){
					System.out.println("Object on left");
					driveTrain.arcadeDrive(0, -ROTATION_SPEED, false);
					Timer.delay(0.05);
				}

				//	If the Pixy sees the object on the right side of robot, turn right
				else if(pkt.x > CENTER_POSITION + CENTER_THRESHOLD){
					System.out.println("Object on right");
					driveTrain.arcadeDrive(0, ROTATION_SPEED, false);
					Timer.delay(0.05);
				}

				//	Refresh the data
				pkt = i2c.getPixy();

				//	Print the data for reference
				System.out.println("XPos: " + pkt.x + "  YPos:  " + pkt.y);

			}

			//	If the object is in the center
			else if(pkt.x > CENTER_POSITION - CENTER_THRESHOLD || pkt.x < CENTER_POSITION + CENTER_THRESHOLD){
				System.out.println("Object is in center");

				//	Drives forward
				driveTrain.arcadeDrive(DRIVE_SPEED,0, false);
				Timer.delay(0.05);

				System.out.println("XPos:  " + pkt.x + "  YPos:   " + pkt.y);

			}
		}

		//  Stops the robot if data is lost or does not exist
		else if(pkt.x == -1){
			driveTrain.arcadeDrive(0, 0, false);
			System.out.println("Data does not exist");
		}

	}

}