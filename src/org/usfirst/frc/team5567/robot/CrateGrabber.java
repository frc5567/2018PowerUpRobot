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
	
	//	Declaring Double Solenoids for arm movement
	DoubleSolenoid dSolLeft;
	DoubleSolenoid dSolRight;
	DoubleSolenoid dSolArm;
	
	//	Declaring intake and launch speed variables
	double intakeSpeed;
	double launchSpeed;
	
	//	Declaring drive train used to control arm wheels
	DifferentialDrive driveTrain;
	
	//	Declaring switch plate for detecting cube
	SwitchPlate armSwitch;
	
	public CrateGrabber(int leftMotorArm, int rightMotorArm, int forwardPortLeft, int backwardPortLeft, int forwardPortRight, int backwardPortRight,
			int forwardPortArm, int backwardPortArm, double cubeIntakeSpeed, double cubeLaunchSpeed){
		//	Instantiating arm wheel speed controllers
		leftArmMotor = new PWMTalonSRX(leftMotorArm);
		rightArmMotor = new PWMTalonSRX(rightMotorArm);
		
		//	Instantiating drive train
		driveTrain = new DifferentialDrive(leftArmMotor, rightArmMotor);
		
		//	Instantiating solonoid
		dSolLeft = new DoubleSolenoid(forwardPortLeft, backwardPortLeft);
		dSolRight = new DoubleSolenoid(forwardPortRight, backwardPortRight);
		dSolArm = new DoubleSolenoid(forwardPortArm, backwardPortArm);
		
		//	Instantiating switchPlate
		armSwitch = new SwitchPlate(0);
	}
	
	//	Method for opening the arms
	public void openGrabber(boolean openGrabber){
		dSolLeft.set(Value.kForward);
		//dSolRight.set(Value.kForward);
	}
	
	//	Method for closing the arms
	public void closeGrabber(boolean closeGrabber){
		dSolLeft.set(Value.kReverse);
		//dSolRight.set(Value.kForward);
	}
	
	//	Method for puling in cubes
	public void cubeIntake(){
		driveTrain.tankDrive(intakeSpeed, intakeSpeed, false);
	}
	
	//	Method for launching cubes
	public void launchCube(boolean launchButton){
		driveTrain.tankDrive(-intakeSpeed, -intakeSpeed, false);
	}
	
	//	Stop intake
	public void stopIntake(){
		driveTrain.tankDrive(0, 0, false);
	}
	
	//	Method for detecting when a cube is in our grasp
	public boolean detectCube(){
		return armSwitch.get();
	}
	
	//	Method for raising arm
	public void raiseArm(){
		dSolArm.set(Value.kForward);
	}
	
	//	Method for lowering arm
	public void lowerArm(){
		dSolArm.set(Value.kReverse);
	}
}
