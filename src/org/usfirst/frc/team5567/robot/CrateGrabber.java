package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class CrateGrabber {
	//	Declaring arm wheel speed controllers
	SpeedController leftArmMotor;
	SpeedController rightArmMotor;
	
	//	Declaring Double Solenoid for arm movement
	DoubleSolenoid dSol;
	
	//	Declaring intake and launch speed variables
	double intakeSpeed;
	double launchSpeed;
	
	//	Declaring drive train used to control arm wheels
	DifferentialDrive driveTrain;
	
	public CrateGrabber(int leftMotorArm, int rightMotorArm, int forwardPort, int backwardPort, double cubeIntakeSpeed, double cubeLaunchSpeed){
		//	Instantiating arm wheel speed controllers
		leftArmMotor = new PWMTalonSRX(leftMotorArm);
		rightArmMotor = new PWMTalonSRX(rightMotorArm);
		
		//	Instantiating drive train
		driveTrain = new DifferentialDrive(leftArmMotor, rightArmMotor);
		
		//	Instantiating solonoid
		dSol = new DoubleSolenoid(forwardPort, backwardPort);
	}
	
	//	Method for opening the arms
	public void openGrabber(boolean openGrabber){
		dSol.set(Value.kReverse);
	}
	
	//	Method for closing the arms
	public void closeGrabber(boolean closeGrabber){
		
	}
}
