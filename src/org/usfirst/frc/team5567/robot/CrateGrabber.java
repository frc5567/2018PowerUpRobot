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
	
	//	Declaring drive train used to control arm wheels
	DifferentialDrive driveTrain;
	
	//	Declaring switch plate for detecting cube
	SwitchPlate armSwitch;
	
	/**
	 * Constructor for the Crate Grabber. Sets speeds and stuff.
	 * 
	 * @param leftMotorArm Port number for the PWMTalonSRX controlling the left arm.
	 * @param rightMotorArm Port number for the PWMTalonSRX controlling the right arm.
	 * @param forwardPortLeft Port number for moving the left Double Solenoid forwards.
	 * @param backwardPortLeft Port number for moving the left Double Solenoid backwards.
	 * @param forwardPortRight Port number for moving the right Double Solenoid forwards.
	 * @param backwardPortRight Port number for moving the right Double Solenoid backwards.
	 * @param forwardPortArm Port number for moving the vertical Double Solenoid up.
	 * @param backwardPortArm Port number for moving the vertical Double Solenoid down.
	 */
	
	public CrateGrabber(int leftMotorArm, int rightMotorArm, int forwardPortLeft, int backwardPortLeft, int forwardPortRight, int backwardPortRight,
			int forwardPortArm, int backwardPortArm){
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
	
	/**
	 * A method for intaking cubes
	 * @param intakeSpeed The speed at which we want cubes to be taken in
	 */
	public void cubeIntake(double intakeSpeed){
		driveTrain.tankDrive(intakeSpeed, intakeSpeed, false);
	}
	
	/**
	 * A method for launching cubes
	 * Launch speed is inverted in class to cause the motors to spin the reverse of intake
	 * @param launchSpeed The speed at which we want the robot to launch cubes, 0 to 1
	 */
	public void launchCube(double launchSpeed){
		driveTrain.tankDrive(-launchSpeed, -launchSpeed, false);
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
