/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	//global variables

	//  Declaring drivetrain Speed Controllers
	final SpeedController frontLeftMotor;
	final SpeedController frontRightMotor;
	final SpeedController backLeftMotor;
	final SpeedController backRightMotor;

	//  Declaring Speed Controller Groups
	final SpeedControllerGroup leftMotors;
	final SpeedControllerGroup rightMotors;

	//  Declaring Encoders for drivetrain motor control
//	final Encoder leftEncoder;
//	final Encoder rightEncoder;

	//  Declaring Xbox controllers for controlling robot
	final XboxController pilotController;
//	final XboxController copilotController;

	//  Declaring Drivetrain for moving the robot
	final DifferentialDrive driveTrain;

	//   Declaring analog Gyro
	final ADXRS450_Gyro myGyro;

	//  Declaring timer used in auto
//	Timer autoTimer;

	//  Declaring Ultrasonics used in auto
//	Ultrasonic lUltra;
//	Ultrasonic rUltra;
	
	//  Declaring Pixy camera used for vision
//	PixyExample myPixy;


	/*
	 * This is our robot's constructor.
	 */
	public Robot() {
		//  Instantiating Speed Controllers and assigned ports
		frontLeftMotor = new VictorSP(0);
		backLeftMotor = new VictorSP(1);
		backLeftMotor.setInverted(true);
		frontRightMotor = new VictorSP(2);
		frontRightMotor.setInverted(false);
		backRightMotor = new VictorSP(3);
		backRightMotor.setInverted(false);

		//  Instantiating Speed Controller Groups
		leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
		rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

		
		//  Instantiating Drivetrain Encoders and assigned ports
//		leftEncoder = new Encoder(6, 7);
//		rightEncoder = new Encoder(8, 9);
//
//		//  Instantiating Xbox Controllers
		pilotController = new XboxController (0);
//		copilotController = new XboxController (1);

		//  Instantiating drivetrain
		driveTrain = new DifferentialDrive(leftMotors, rightMotors);

		//  Instantiating and calibrating analog gyro
		myGyro = new ADXRS450_Gyro();
		myGyro.calibrate();

		//  Instantiating ultrasonics
//		lUltra = new Ultrasonic(1,0);
//		rUltra = new Ultrasonic(3,2);
		
		//  Instantiating pixy camera
//		myPixy = new PixyExample();

	}






	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {


//		lUltra.setEnabled(true);
//		rUltra.setEnabled(true);


	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
//		lUltra.setAutomaticMode(true);
//		rUltra.setAutomaticMode(true);
//		//  Instantiating, reseting, and starting timer used in timer
//		autoTimer = new Timer();
//		autoTimer.reset();
//		autoTimer.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

		//  Auton for testing vision
//		// Prevents the robot from moving if it's too close to the wall
//		if((lUltra.getRangeMM()>150) && rUltra.getRangeMM()>150){
//			// A method that turns the robot to face the target
//			myPixy.centerOnObject(driveTrain);
//			Timer.delay(.05);
//		}
		/*  Commented out for easy of testing auton
		//  Drives forward when timer is less than 5 seconds
		if(autoTimer.get() < 5) {
			driveTrain.arcadeDrive(0.5, 0);
		}
		//stop when timer is after 5 seconds
		else {
			driveTrain.arcadeDrive(0, 0);
			//Rotate using gyro data 180 degrees
			if(myGyro.getAngle() < 180) {
				driveTrain.arcadeDrive(0, .5);
			}
			else {
				driveTrain.arcadeDrive(0, 0);
			}
		}
		*/
	}

	public void teleopInit() {
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		driveTrain.tankDrive(-pilotController.getY(Hand.kLeft), -pilotController.getY(Hand.kRight));
	}

	
	public void testInit(){
		System.out.println("here");
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		driveTrain.tankDrive(.75, .75);
//		System.out.println("gyro:\t "+myGyro.getAngle());
		Timer.delay(.1);
	}
}
