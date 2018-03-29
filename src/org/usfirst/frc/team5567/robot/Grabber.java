package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public abstract class Grabber implements MotorSafety {

	//	Constants for arm position/rotation
	double armEncRaised = -170;
	double armEncInit = -170;
	final double kArmEncIntBack = -180;
	final double kArmEncLower = 0;
	
	DigitalInput outerBreakBeam;
	DigitalInput innerBreakBeam;
	
	//	Declares Enum for state of pneumatic arm for opening and closing CrateGrabber Arm
	public enum ArmState {
		kOpen(0),
		kClosed(1);

		private int armValue;

		ArmState(int armValue) {
			this.armValue = armValue;
		}
	}

	//	Declares Enum for state of pneumatic arm for raising and lowering CrateGrabber Arm
	public enum AngleState {
		kInitial(0),
		kRaised(1),
		kLowered(2);

		private int angleValue;

		AngleState(int angleValue) {
			this.setAngleValue(angleValue);
		}

		/**
		 * @return the angleValue
		 */
		public int getAngleValue() {
			return angleValue;
		}

		/**
		 * @param angleValue the angleValue to set
		 */
		public void setAngleValue(int angleValue) {
			this.angleValue = angleValue;
		}
	}

	//	Declares Enum for state of pneumatic arm for running the motors to intaking and deposit a cube on the CrateGrabber Arm
	public enum MotorState {
		kOff(0),
		kIntake(1),
		kDeposit(2);

		private int motorValue;

		MotorState(int motorValue) {
			this.motorValue = motorValue;
		}
	}
	
	// Enum for whether we are working in automatic or manual cube intake
	public enum IntakeMode {
		kAutomatic(0),
		kManual(1);
		
		private int intakeMode;
		
		IntakeMode(int inMode) {
			this.intakeMode = inMode;
		}
	}

	//	Declares motor safety helper
	protected MotorSafetyHelper m_safetyHelper;

	//	Declares encoder for monitoring arm
	Encoder armEncoder;

	//	Declaring arm wheel speed controllers
	SpeedController leftArmMotor;
	SpeedController rightArmMotor;
	SpeedController raiseArmMotor;

	//	Declaring Double Solenoids for arm movement
	DoubleSolenoid dSolLeft;
	DoubleSolenoid dSolRight;
	DoubleSolenoid dSolArm;

	//	Declaring drive train used to control arm wheels
	DifferentialDrive driveTrain;

	//	Declaring switch plate for detecting cube
	DigitalInput armSwitch;

	/**
	 * Constructor for the Crate Grabber. Sets speeds and stuff.
	 * 
	 * @param leftMotorArm Port number for the PWMTalonSRX controlling the left arm.
	 * @param rightMotorArm Port number for the PWMTalonSRX controlling the right arm.
	 * @param forwardPortLeft Port number for moving the left Double Solenoid forwards.
	 * @param backwardPortLeft Port number for moving the left Double Solenoid backwards.
	 * @param forwardPortRight Port number for moving the right Double Solenoid forwards. DONT USE
	 * @param backwardPortRight Port number for moving the right Double Solenoid backwards. DONT USE
	 * @param forwardPortArm Port number for moving the vertical Double Solenoid up. DONT USE
	 * @param backwardPortArm Port number for moving the vertical Double Solenoid down. DONT USE
	 * @param raiseArmMotorPort Port number for the raise arm motor
	 */

	public Grabber(int leftMotorArm, int rightMotorArm, int forwardPortLeft, int backwardPortLeft){
		//	Instantiating arm wheel speed controllers
		leftArmMotor = new PWMTalonSRX(leftMotorArm);
		rightArmMotor = new PWMTalonSRX(rightMotorArm);

		//	Instantiating drive train
		driveTrain = new DifferentialDrive(leftArmMotor, rightArmMotor);

		//	Instantiating solenoid
		dSolLeft = new DoubleSolenoid(forwardPortLeft, backwardPortLeft);
		//dSolRight = new DoubleSolenoid(forwardPortRight, backwardPortRight);
		//dSolArm = new DoubleSolenoid(forwardPortArm, backwardPortArm);

		// InfraRed Breakbeam sensors for cube grabber
		outerBreakBeam = new DigitalInput(11); // Port 1 on MXP breakout on NavX IMU
		innerBreakBeam = new DigitalInput(10); // Port 0 on MXP breakout on NavX IMU

		//	Instantiates encoder for monitoring/controlling arm
		armEncoder = new Encoder(2, 1, false, EncodingType.k1X);

		//	Sets up motor safety
		setupMotorSafety();
	}

	/**Method to set the solenoid to open and close the CrateGrabber Arm
	 * @param armValue The enum passed to set the solenoid value 
	 */
	public void setGrabberArm(ArmState armValue) {
		switch (armValue) {
		case kOpen:
			dSolLeft.set(Value.kForward);
			break;
		case kClosed:
			dSolLeft.set(Value.kReverse);
			break;	
		}

	}

	/*	//	Method for opening the arms
	public void openGrabber(boolean openGrabber){
		dSolLeft.set(Value.kForward);
		//dSolRight.set(Value.kForward);
	}

	//	Method for closing the arms
	public void closeGrabber(boolean closeGrabber){
		dSolLeft.set(Value.kReverse);
		//dSolRight.set(Value.kForward);
	}*/

	public void setMotorArm(MotorState motorValue, double intakeSpeed, double cubeLaunchSpeed) {
		switch(motorValue) {
		case kOff:
			driveTrain.tankDrive(0, 0, false);
			break;
		case kIntake:
			driveTrain.tankDrive(intakeSpeed, intakeSpeed, false);
			break;
		case kDeposit:
			driveTrain.tankDrive(.75*(-cubeLaunchSpeed), .75*(-cubeLaunchSpeed), false);
			break;
		}
	}

	/*
	 * A method for intaking cubes
	 * @param currentIntakeMode Cube intake mode of operation -- auto or manual
	 * @param intakeSpeed Cube intake speed
	 */
	public boolean cubeIntake(IntakeMode currentIntakeMode, double intakeSpeed){
		boolean gotCube = false;
		
		//	Puts the grabber in automatic mode only when desired and the grabber is lowered
		if (currentIntakeMode == IntakeMode.kAutomatic && armEncoder.getRaw() > -15){
			// Check IR Break Beams
			// if outer beam is broken, make sure grabber arm closes and engage motors
			if (!outerBreakBeam.get()) {
				this.setGrabberArm(ArmState.kClosed);
				this.setMotorArm(MotorState.kIntake, intakeSpeed, 0.0);
				
			} else {// if outer beam is NOT broken, make sure grabber arm is open and motors off
				this.setGrabberArm(ArmState.kOpen);
				this.setMotorArm(MotorState.kIntake, 0.25, 0.0);			
			}
			
			// if inner beam is broken, make sure to stop intake motors.
			if (!innerBreakBeam.get()) {
				// Try to hold in the cube -- set this to 0.0 if we have issues with it.
				this.setMotorArm(MotorState.kOff, 0.05, 0.0);	
				gotCube = true;
			}
		} else {
			this.setMotorArm(MotorState.kIntake, 0.05, 0.0);
		}
		return gotCube;
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

	public abstract void setAngleArm(AngleState angleValue, double speed);

	//	Below lies the required motor safety methods
	@Override
	public void setExpiration(double timeout) {
		m_safetyHelper.setExpiration(timeout);		
	}

	@Override
	public double getExpiration() {
		return m_safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
		return m_safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		if (raiseArmMotor != null) {
			raiseArmMotor.stopMotor();
		}
		if (m_safetyHelper != null) {
			m_safetyHelper.feed();
		}
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
		m_safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
		return m_safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		// TODO Auto-generated method stub
		return "CrateGrabber";
	}

	private void setupMotorSafety() {
		m_safetyHelper = new MotorSafetyHelper(this);
		m_safetyHelper.setExpiration(.1);
		m_safetyHelper.setSafetyEnabled(true);
	}
}