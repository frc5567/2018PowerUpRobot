package org.usfirst.frc.team5567.robot;


//	Combines all the drive methods used in auto into here to save space in the main robot class
/**
 * @author Matt
 *
 */
public class AutoDrive extends Robot{

	//	Might be deprecated since we use StraightDriveAngle
	/* Method for driving straight in auton
	 * @param targetDistance The distance you want to travel (in inches)
	 * @param speed The speed that you want the robot to travel at (range is -1 to 1)
	 */
	public void StraightDrive(double targetDistance, double speed){

		//	Resets the encoders and the distance traveled the first time this enters
		if(firstFlag){
			leftEncoder.reset();
			rightEncoder.reset();

			rDistance = 0;
			lDistance = 0;

			straightController.reset();
			straightController.enable();

			System.out.println("resetting");

			// Sets the Setpoint so the robot travels straight
			straightController.setSetpoint(0);

			firstFlag = false;
		}

		//	Enables the turn controller if it is not already
		if (!straightController.isEnabled()) {
			rotateToAngleRate = 0;
			straightController.enable();
		}

		//	Gets the total distance from the encoders
		//	This encoder must be inverted so that we get proper values
		rDistance = rightEncoder.getDistance();
		lDistance = leftEncoder.getDistance();

		//	Prints distance from encoders
		//		System.out.println(rDistance + "   " + lDistance);

		//	Gets rate of rotation from PID
		rotateToAngleRate = straightController.get();

		//	Stops robot if target distance was reached and moves to the next case
		if(targetDistance <= lDistance || targetDistance <= rDistance){
			driveTrain.arcadeDrive(0, 0, false);
			autoCase++;
			firstFlag = true;
			rDistance = 0;
			lDistance = 0;
		}

		//	Drives straight forward if target is not reached
		else{
			driveTrain.arcadeDrive(speed, rotateToAngleRate, false);
		}
	}
	
	//	This is what we use currently
	/* Method for driving straight in auton
	 * @param targetDistance The distance you want to travel (in inches)
	 * @param speed The speed that you want the robot to travel at (range is -1 to 1)
	 * @param driveAngle Angle that is used as "zero" when going straight after turning
	 */
	public void StraightDriveAngle(double targetDistance, double speed, double driveAngle){

		//	Resets the encoders and the distance traveled the first time this enters
		if(firstFlag){
			leftEncoder.reset();
			rightEncoder.reset();

			rDistance = 0;
			lDistance = 0;

			System.out.println("resetting");

			straightController.reset();
			straightController.enable();

			// Sets the Setpoint so the robot travels straight
			straightController.setSetpoint(driveAngle);

			firstFlag = false;
		}

		//	Enables the turn controller if it is not already
		if (!straightController.isEnabled()) {
			rotateToAngleRate = 0;
			straightController.enable();
		}

		//	Gets the total distance from the encoders
		//	This encoder must be inverted so that we get proper values
		rDistance = rightEncoder.getDistance();
		lDistance = leftEncoder.getDistance();

		//	Prints distance from encoders
		//		System.out.println("R:[" +rDistance+ "][" +rightEncoder.getRaw()+ "] L:[" +lDistance+ "][" +leftEncoder.getRaw()+ "]");

		//	Gets rate of rotation from PID
		rotateToAngleRate = straightController.get();

		//	Stops robot if target distance was reached and moves to the next case
		if(targetDistance <= lDistance /*|| targetDistance <= rDistance*/){
			driveTrain.arcadeDrive(0, 0, false);

			turnController.reset();
			System.out.println("here");
			autoCase++;
			firstFlag = true;
			rDistance = 0;
			lDistance = 0;
		}

		//	Drives straight forward if target is not reached
		else{
			driveTrain.arcadeDrive(speed, rotateToAngleRate, false);
		}
	}
	
	/* Method that rotates in place to a given angle in auto
	 * @param targetAngle The angle we want to turn to (in degrees)
	 */
	public void RotateDrive(double targetAngle){
		//  If this is the first time entering this method, sets target angle
		if(firstFlag){
			turnController.reset();
			rotateCount = 0;
			turnController.setSetpoint(targetAngle);

			turnController.reset();
			turnController.enable();

			firstFlag = false;
		}

		//	 If the turn controller is not enabled, enable turn controller
		if (!turnController.isEnabled()) {
			//	rotateRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}

		//	Sets the speed the the robot rotates at from the PID
		rotateToAngleRate = turnController.get();

		//	Prints setpoint and rotation rate
		//		System.out.println(turnController.getSetpoint());
		//		System.out.println(rotateToAngleRate);
		//		System.out.println(ahrs.getAngle());

		/*if(-rotateThreshold < rotateToAngleRate && rotateToAngleRate < rotateThreshold) {
			rotateCount++;
		}

		//	If the PID has slowed down to a certain point, exit the case
		if((rotateToAngleRate < rotateThreshold && rotateToAngleRate > -rotateThreshold) && rotateCount > 4){
			//	If we have, stop and return true
			autoCase++;
			firstFlag = true;
		}*/

		//	Makes the robot turn to angle
		driveTrain.arcadeDrive(0, rotateToAngleRate, false);
	}
	
}
