package org.usfirst.frc.team5567.robot;

import org.usfirst.frc.team5567.robot.PixyPacket;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
public class PixyExample {

	// Sets up the I2C interface
	M_I2C i2c = new M_I2C();

	// Creates a Pixy packet to transfer Pixy data
	PixyPacket pkt = i2c.getPixy();

	/*
	 * This method will turn the robot to face the object the Pixy cam has locked on to
	 */
	public void centerOnObject(DifferentialDrive driveTrain){

		// If data exists, proceed (-1 is the default value)
		if(pkt.x != -1){

			// The code on the arduino decides what object to send

			// While the object is not center
			if(pkt.x < .48 || pkt.x > .52){

				// If the Pixy sees the object on the left side of robot, turn left
				if(pkt.x < .48){

					driveTrain.arcadeDrive(0, -0.2);
				}

				// If the Pixy sees the object on the right side of robot, turn right
				else if(pkt.x > .52){

					driveTrain.arcadeDrive(0, 0.2);
				}

				// Refresh the data
				pkt = i2c.getPixy();

				// Print the data for reference
				System.out.println("XPos: " + pkt.x);

			}

			// If the object is in the center
			if(pkt.x > .48 || pkt.x < .52){

				// Drives forward
				driveTrain.arcadeDrive(0.3,0);

			}
		}

	}

}
