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
		//	Inverted the motor for the test bot
		raiseArmMotor.setInverted(true);
	}

	@Override
	public void setAngleArm(AngleState angleValue, double speed){
		switch(angleValue) {
		case kInitial:
//			Reversed signs for test bot - originally <, >
			if(armEncoder.getRaw() > kArmEncInt-3){ //&& armEncoder.getRaw() >= kArmEncIntBack) {
				raiseArmMotor.set(-speed); //Lowers arms
				System.out.println("armEnc:\t"+armEncoder.getRaw());
//				Robot.autoCase++;
			}
			else if(armEncoder.getRaw() < kArmEncInt+3){//kArmEncIntBack){
				raiseArmMotor.set(speed); //Raises arms
				System.out.println("armEnc:\t"+armEncoder.getRaw());
			}
			else {
				raiseArmMotor.set(0);
//				raiseArmMotor.set(speed);
				System.out.println("armEnc:\t"+armEncoder.getRaw());
			}
			break;
			//	Reversed signs for test bot - originally <, >
			case kRaised:
				if(armEncoder.getRaw() > kArmEncRaised+3) {
					raiseArmMotor.set(-speed);
					System.out.println("armEnc:\t"+armEncoder.getRaw());
//					Robot.autoCase++;
				}
				else if(armEncoder.getRaw() < kArmEncRaised-3) {
					raiseArmMotor.set(speed);
					System.out.println("armEnc:\t"+armEncoder.getRaw());
				}
				else {
					raiseArmMotor.set(0);
					System.out.println("armEnc:\t"+armEncoder.getRaw());
				}
				break;
			case kLowered:
				//	Changed from >= to <= for the test bot inverted
				if(armEncoder.getRaw() <= kArmEncLower) {
					raiseArmMotor.set(0);
					System.out.println("armEnc:\t"+armEncoder.getRaw());
//					Robot.autoCase++;
				}
				else {
					raiseArmMotor.set(speed);
					System.out.println("armEnc:\t"+armEncoder.getRaw());
				}
				System.out.println("Arm Encoder Value \t" + armEncoder.getRaw());
				break;
			}

		}
	}
