package org.usfirst.frc.team5567.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
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

/**
 * This is a demo program showing the use of the navX MXP to implement
 * the "rotate to angle", "zero yaw" and "drive straight" on a Tank
 * drive system.
 *
 * If Left Joystick Button 0 is pressed, a "turn" PID controller will 
 * set to point to a target angle, and while the button is held the drive
 * system will rotate to that angle (NOTE:  tank drive systems cannot simultaneously
 * move forward/reverse while rotating).
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Finally, if Left Joystick button 2 is held, the "turn" PID controller will
 * be set to point to the current heading, and while the button is held,
 * the driver system will continue to point in the direction.  The robot 
 * can drive forward and backward (the magnitude of motion is the average
 * of the Y axis values on the left and right joysticks).
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */

public class Robot extends IterativeRobot implements PIDOutput {
	//	global variables

	//  Declaring drivetrain Speed Controllers
	final SpeedController frontLeftMotor;
	final SpeedController frontRightMotor;
	final SpeedController backLeftMotor;
	final SpeedController backRightMotor;

	//  Declaring Speed Controller Groups
	final SpeedControllerGroup leftMotors;
	final SpeedControllerGroup rightMotors;
	
	//  Declaring Xbox controllers for controlling robot
	final XboxController pilotController;
	
	//  Declaring Drivetrain for moving the robot
	final DifferentialDrive driveTrain;

	AHRS ahrs;

	PIDController turnController;
	double rotateToAngleRate;

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system.  Note that the      */
	/* SmartDashboard in Test mode has support for helping you tune    */
	/* controllers by displaying a form where you can enter new P, I,  */
	/* and D constants and test the mechanism.                         */

	static final double kP = 0.015;
	static final double kI = 0.00;
	static final double kD = 0.035;
	static final double kF = 0.00;

	static final double kToleranceDegrees = 2.0f;    

	static final double kTargetAngleDegrees = 90.0f;

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

		//			//  Instantiating Xbox Controllers
		pilotController = new XboxController (0);

		//  Instantiating drivetrain
		driveTrain = new DifferentialDrive(leftMotors, rightMotors);


		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
			ahrs = new AHRS(SPI.Port.kMXP); 
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		turnController.disable();

		/* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		/* tuning of the Turn Controller's P, I and D coefficients.            */
		/* Typically, only the P value needs to be modified.                   */
//		LiveWindow.addActuator("DriveSystem", "RotateController", turnController);        
	}

	public void teleopInit(){
		ahrs.zeroYaw();
	}
	
	/**
	 * Runs the motors with tank steering.
	 */
	public void teleopPeriodic() {
		driveTrain.setSafetyEnabled(true);
			if (true) {
				/* While this button is held down, rotate to target angle.  
				 * Since a Tank drive system cannot move forward simultaneously 
				 * while rotating, all joystick input is ignored until this
				 * button is released.
				 */
				if (!turnController.isEnabled()) {
					turnController.setSetpoint(0);
					rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
					turnController.enable();
				}
				driveTrain.arcadeDrive(pilotController.getY(Hand.kLeft), rotateToAngleRate, false);
			}/* else if ( pilotController.getBButton()) {
				 "Zero" the yaw (whatever direction the sensor is 
				 * pointing now will become the new "Zero" degrees.
				 
				ahrs.zeroYaw();
			} else if ( pilotController.getRawButton(2)) {
				 While this button is held down, the robot is in
				 * "drive straight" mode.  Whatever direction the robot
				 * was heading when "drive straight" mode was entered
				 * will be maintained.  The average speed of both 
				 * joysticks is the magnitude of motion.
				 
				if(!turnController.isEnabled()) {
					// Acquire current yaw angle, using this as the target angle.
					turnController.setSetpoint(ahrs.getYaw());
					rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
					turnController.enable();
				}
				double magnitude = (pilotController.getY(Hand.kLeft) + pilotController.getY(Hand.kRight)) / 2;
				double leftStickValue = magnitude + rotateToAngleRate;
				double rightStickValue = magnitude - rotateToAngleRate;
				driveTrain.tankDrive(leftStickValue,  rightStickValue);
			} else {
				 If the turn controller had been enabled, disable it now. 
				if(turnController.isEnabled()) {
					turnController.disable();
				}
				 Standard tank drive, no driver assistance. 
				driveTrain.tankDrive(pilotController.getY(Hand.kLeft), pilotController.getY(Hand.kRight));
			}*/
			Timer.delay(0.05);		// wait for a motor update time
	}

	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX MXP yaw angle input and PID Coefficients.    */
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
	
	public void testInit(){
		ahrs.zeroYaw();
		if (!turnController.isEnabled()) {
			turnController.setSetpoint(kTargetAngleDegrees);
			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}
	}
	public void testPeriodic(){
		driveTrain.arcadeDrive(0, rotateToAngleRate, false);
	}
}