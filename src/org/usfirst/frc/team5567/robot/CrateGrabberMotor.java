package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.VictorSP;


public class CrateGrabberMotor extends Grabber {

	/**
	 * 	A crate grabber raised by a motor
	 * @param leftMotorArm	The PWM port for the left motor on the arm
	 * @param rightMotorArm	The PWM port for the right motor on the arm
	 * @param forwardPortLeft	The forward port for the opening and closing solonoid
	 * @param backwardPortLeft	The backward port for the opening and closing solonoid
	 * @param raiseArmMotorPort	The PWM port for the motor that raises the arm
	 */
	public CrateGrabberMotor(int leftMotorArm, int rightMotorArm, int forwardPortLeft, int backwardPortLeft,
			int raiseArmMotorPort) {
		super(leftMotorArm, rightMotorArm, forwardPortLeft, backwardPortLeft);
		raiseArmMotor = new VictorSP(raiseArmMotorPort);
	}

	@Override
	public void setAngleArm(AngleState angleValue, double speed){
		switch(angleValue) {
		case kRaised:
			raiseArmMotor.set(speed);
			break;
		case kLowered:
			raiseArmMotor.set(-speed);
			break;
		}

	}
}
