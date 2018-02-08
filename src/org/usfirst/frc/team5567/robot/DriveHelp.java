package org.usfirst.frc.team5567.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

public class DriveHelp {

	DifferentialDrive driveTrain;
	PIDController pidControlStraight;
	PIDController pidControlRotate;
	Encoder rightEncoder;
	Encoder leftEncoder;

	AHRS myGyro;

	boolean straightFlag;
	boolean turningFlag;

	DriveHelp(AHRS gyro, DifferentialDrive diffDrive, PIDController pidRotateInput, PIDController pidStraightInput, Encoder rEncoder, Encoder lEncoder){

		//	Storing the parameters in member variables
		driveTrain = diffDrive;
		pidControlStraight = pidStraightInput;	
		pidControlRotate = pidRotateInput;
		rightEncoder = rEncoder;
		leftEncoder = lEncoder;
		myGyro = gyro;
	}

	/*
	 * 	Speed needs to be determined by a seperate PID controller using encoders and then passed in
	 */
	public boolean StraightDrive(double targetDistance, double speed, double rotateRate){
		
		rotateRate = pidControlStraight.get();
		//	read encoder values
		//	determine how far we have traveled
		//	determine whether target distance was reached or not
		if(targetDistance <= leftEncoder.getDistance() || targetDistance <= rightEncoder.getDistance()){
			//			if target distance was reached stop and return true
			driveTrain.arcadeDrive(0, 0, false);
			straightFlag = true;
		}
		else{
			//			if target distance not reached drive forward
			driveTrain.arcadeDrive(speed, rotateRate, false);
			straightFlag = false;
		}
		return straightFlag;
	}

	public boolean RotateDrive(double targetAngle, double rotateRate){
		pidControlRotate.setSetpoint(targetAngle);
		rotateRate = pidControlRotate.get();
		
		//	Read gyro values
		//	Determine if we have turned to target
		if(myGyro.getAngle() >= targetAngle){
			//	If we have, stop and return true
			turningFlag = true;
		}
		
		else{
			turningFlag = false;
			//	If not, turn to angle
		}
		driveTrain.arcadeDrive(0, rotateRate, false);
		return turningFlag;
	}




}
