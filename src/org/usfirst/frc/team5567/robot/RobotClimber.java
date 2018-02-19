package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.VictorSP;


public class RobotClimber {
	
	//	Declares Enum for state of the pneumatics for the climber
	public enum ClimbState {
		kOff(0),
		kExtend(1),
		kRetract(2);
		
		private int climbValue;
		
		ClimbState(int climbValue) {
			this.climbValue = climbValue;
		}
	}
	
	//	Declaring winch motors
	SpeedController winchMotor1;
	SpeedController winchMotor2;
	
	
	//	Declaring double Solenoid
	DoubleSolenoid climberDSol;
	
	//	Declaring drive train used to control winch
	final DifferentialDrive winchDrive;
	
	
	/**
	 * Constructor for the climber. Instantiates the winch motors and the Solenoid ports.
	 * 
	 * @param winchMotor1Port Port number for the first winch motor.
	 * @param winchMotor2Port Port number for the second winch motor.
	 * @param forwardPort Port number for moving the Solenoid forwards.
	 * @param backwardPort Port number for moving the Solenoid backwards.
	 */
	public RobotClimber(int winchMotor1Port, int winchMotor2Port, int forwardPort, int backwardPort){
		
		//	Instantiating motors based off of ports from the constructor
		winchMotor1 = new VictorSP(winchMotor1Port);
		winchMotor2 = new VictorSP(winchMotor2Port);
		
		climberDSol = new DoubleSolenoid(forwardPort, backwardPort);
		
		// DifferentialDrive requires that you invert a speed controller before passing in, if appropriate.
		// TODO: Need to test which side needs to be inverted
		winchMotor1.setInverted(false);
		winchMotor2.setInverted(false);
		
		//	Instantiating a drive train to be used as a winch
		winchDrive = new DifferentialDrive(winchMotor1, winchMotor2);
		
	}
	
	/**
	 * @param climbValue The Enum for setting the state for the pneumatics on the climber arm
	 */
	public void setClimbSolenoid(ClimbState climbValue) {
		switch(climbValue) {
			case kOff:
				climberDSol.set(Value.kOff);
				break;
			case kExtend:
				climberDSol.set(Value.kForward);
				break;
			case kRetract:
				climberDSol.set(Value.kReverse);
				break;
		}
	}	
	
	/**
	 * Allows us to control the winch speed, at full speed or half speed.
	 * 
	 * @param driveSpeed The raw input from a trigger/stick that runs the motors.
	 * @param halfSpeed If this is true, meaning the button is pressed, the motors will run at half the input speed.
	 */
	public void winchControl(double driveSpeed, boolean halfSpeed){
		
		if(halfSpeed = false){
			
			winchDrive.arcadeDrive(driveSpeed, 0, false);
		}
		
		
		else if (halfSpeed) {			
			
			winchDrive.arcadeDrive(driveSpeed/2, 0, false);
		}
		
	}
}
