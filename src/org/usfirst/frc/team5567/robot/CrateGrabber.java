/**
 * 
 */
package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Victor;
/**
 * @author Matt & Quinn
 *
 */
public class CrateGrabber {


	//	Declaring the double solenoid for arm pneumatics
	DoubleSolenoid mainDoubleSolenoid;

	//  Declares speed controllers for the motors on arm
	SpeedController leftArmController;
	SpeedController rightArmController;


	/**
	 * Constructor that initializes the pneumatics required for the arm
	 * @param forewardChannel The channel on the PCM for actuator's forward
	 * @param reverseChannel The channel on the PCM for actuator's reverse
	 * @param leftArmMotor The channel on the RIO's PWM port for the left motor
	 * @param rightArmMotor The channel on the RIO's PWM port for the right motor
	 */
	public CrateGrabber(int forwardChannel, int reverseChannel, int leftArmMotor, int rightArmMotor) {
		// TODO Auto-generated constructor stub

		//	Instantiating the double solenoid and assigning ports
		mainDoubleSolenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
		
		//Instantiating the motor controllers for the motors on the end of the box grabber
		leftArmController = new Victor(leftArmMotor);
		rightArmController = new Victor(rightArmMotor);

	}
	
	public void boxArm(boolean boxSwitch, boolean isOpen, boolean isClosed) {
		if((boxSwitch && !isOpen) || isClosed) {
			mainDoubleSolenoid.set(Value.kForward);
		}
	}
	
	
}
