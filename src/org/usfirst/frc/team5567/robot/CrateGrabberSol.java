package org.usfirst.frc.team5567.robot;

import org.usfirst.frc.team5567.robot.Grabber.AngleState;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class CrateGrabberSol extends Grabber{

	/**
	 * 	A crate grabber raised by a solonoid
	 * @param leftMotorArm	The PWM port for the left motor on the arm
	 * @param rightMotorArm	The PWM port for the right motor on the arm
	 * @param forwardPortLeft	The forward port for the opening and closing solonoid
	 * @param backwardPortLeft	The backward port for the opening and closing solonoid
	 * @param forwardPortArm	The forward port for the raising and lowering solonoid
	 * @param backwardPortArm The backward port for the raising and lowering solonoid
	 */
	public CrateGrabberSol(int leftMotorArm, int rightMotorArm, int forwardPortLeft, int backwardPortLeft,
			int forwardPortArm, int backwardPortArm) {
		super(leftMotorArm, rightMotorArm, forwardPortLeft, backwardPortLeft);
		
		dSolArm = new DoubleSolenoid(forwardPortArm, backwardPortArm);
	}

	@Override
	public void setAngleArm(AngleState angleValue, double speed){
		switch(angleValue) {
		case kRaised:
			dSolArm.set(Value.kForward);
			break;
		case kLowered:
			dSolArm.set(Value.kReverse);
			break;
		}

	}

}
