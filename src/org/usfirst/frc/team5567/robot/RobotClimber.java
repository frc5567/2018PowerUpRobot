package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class RobotClimber {
	
	//	Declaring winch motors
	SpeedController winchMotor1;
	SpeedController winchMotor2;
	
	
	//	Declaring double Solenoid
	DoubleSolenoid climberDSol;
	
	//	It's that thing
	final DifferentialDrive winchDrive;
	
	
	/**
	 * Constructor for the climber. Instantiates the winch motors and the Solenoid ports.
	 * 
	 * @param winchMotor1Port Port number for the first winch motor.
	 * @param winchMotor2Port Port number for the sencond winch motor.
	 * @param forwardPort Port number for moving the Solenoid forwards.
	 * @param backwardPort Port number for moving the Solenoid backwards.
	 */
	public RobotClimber(int winchMotor1Port, int winchMotor2Port, int forwardPort, int backwardPort){
		
		winchMotor1 = new Talon(winchMotor1Port);
		winchMotor2 = new Talon(winchMotor2Port);
		
		climberDSol = new DoubleSolenoid(forwardPort, backwardPort);
		
		winchDrive = new DifferentialDrive(winchMotor1, winchMotor2);
		
	}
	
	/**
	 * Extends the Solenoid.
	 */
	public void ExtendSolenoid(){
		climberDSol.set(Value.kForward);
	}
	
	/**
	 * Retracts the Solenoid.
	 */
	public void RetractSolenoid(){
		climberDSol.set(Value.kReverse);
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
		
		else if(halfSpeed = true){
			
			winchDrive.arcadeDrive(driveSpeed/2, 0, false);
		}
		
	}
}
