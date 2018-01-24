package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.can.CANStatus;

/**
 * 	press button to activate wheels spinning inward
 *	sensor detects cube is fully in and deactivates wheels
 *	activates pneumatics on button press to close arms and clamp cube
 *	button press to turn off pneumatics
 *	button press to spin wheels outward to launch cube
 *	button press to activate pneumatics and open arms
 */
@SuppressWarnings("unused")
public class Robot extends IterativeRobot {

	// Sets up the channel for the double solenoid on the PCM
	int forwardChannel = 0;
	int reverseChannel = 1;
	
	//  Creates Pneumatic Arm AND initialize with the PCM ports 
	//  PCM ports set by the forward and reverse channel variables
	CrateGrabber boxArm = new CrateGrabber(forwardChannel, reverseChannel);
	
/*
	//	Declaring arm wheel speed controllers
	SpeedController leftArmMotor;
	SpeedController rightArmMotor;

	//	Declaring speed controller group to power both wheels at the same time
	SpeedControllerGroup armMotors;
	
*/
	//	Declaring Xbox controller for controlling arms
	XboxController pilotControl;

	//	Declaring the double solenoid for arm pneumatics
	DoubleSolenoid dSol;

	public void robotInit() {
	
/*
		//	Instantiating speed controllers and assigning ports
		leftArmMotor = new VictorSP(0);
		rightArmMotor = new VictorSP(1);

		//	Instantiating speed controller groups
		armMotors = new SpeedControllerGroup(leftArmMotor, rightArmMotor);
*/
		//	Instantiating the double solenoid and assigning ports
		dSol = new DoubleSolenoid(0,1);
		
		// Instantiating Xbox Controller and assigning ports
		pilotControl = new XboxController(0);
	}

	public void TeleopInit() {

	}

	public void teleopPeriodic(){

		
	}












}


