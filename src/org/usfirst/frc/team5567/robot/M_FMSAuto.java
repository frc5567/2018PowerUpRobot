package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class M_FMSAuto extends Robot{

	public void getFMSPosition() {
		//  Gets the string from the FMS that shows which side of the switch and scale
		//  This is mainly a failsafe if the string doesn't get to the robot
		//  This is the failsafe so the fatal error doesn't kill the robot
		try {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		catch (RuntimeException ex ) {
			DriverStation.reportError("Error retrieveing switch and scale data from FMS:  " + ex.getMessage(), true);
			fmsAutoSelected = Default;
		}
		
		//	Gets FMS data, chooses Auto case based on it
		//	If the string returns the way it's supposed to, the variable is set to the side the switch is on
		//	toString function is to change the char back to a STring (What the variable is)
		if(gameData.charAt(0) == 'L' || gameData.charAt(0) == 'R') {
			fmsAutoSelected = Character.toString(gameData.charAt(0));
		}
		//	Another failsafe in case a weird string comes back no matching for Left or Right
		else {
			//	Sets robot to use default code
			fmsAutoSelected = Default;
			System.out.println("Default Auto Selected");
		}
		
	}
	

}
