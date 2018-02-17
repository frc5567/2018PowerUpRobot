package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class CrateGrabber  implements MotorSafety {
	
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
	
	public CrateGrabber(int leftMotorArm, int rightMotorArm, int forwardPortLeft, int backwardPortLeft,/* int forwardPortRight, int backwardPortRight,
			int forwardPortArm, int backwardPortArm*/ int raiseArmMotorPort){
		//	Instantiating arm wheel speed controllers
		leftArmMotor = new PWMTalonSRX(leftMotorArm);
		rightArmMotor = new PWMTalonSRX(rightMotorArm);
		raiseArmMotor = new VictorSP(raiseArmMotorPort);
		
		//	Instantiating drive train
		driveTrain = new DifferentialDrive(leftArmMotor, rightArmMotor);
		
		//	Instantiating solenoid
		dSolLeft = new DoubleSolenoid(forwardPortLeft, backwardPortLeft);
		//dSolRight = new DoubleSolenoid(forwardPortRight, backwardPortRight);
		//dSolArm = new DoubleSolenoid(forwardPortArm, backwardPortArm);
		
		//	Instantiating armswitch
		armSwitch = new DigitalInput(0);
		
		//	Instantiates encoder for montitoring/controlling arm
		armEncoder = new Encoder(2, 1, false, EncodingType.k1X);
		
		//	Sets up motor safety
		setupMotorSafety();
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
	public void raiseArm(double speed){
		if(armEncoder.getRaw() >= 20){
			raiseArmMotor.set(0);
		}
		else{
		raiseArmMotor.set(speed);
		}
		//dSolArm.set(Value.kForward);
	}
	
	//	Method for lowering arm
	public void lowerArm(double speed){
		if(armEncoder.getRaw() <= 0){
			raiseArmMotor.set(0);
		}
		else{
		raiseArmMotor.set(-speed);
		}
		//dSolArm.set(Value.kReverse);
	}

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
