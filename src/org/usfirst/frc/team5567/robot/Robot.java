package org.usfirst.frc.team5567.robot;

import edu.wpi.first.wpilibj.DriverStation;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.CameraServer;

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

//  This is the main robot that we use in competition.
public class Robot extends IterativeRobot implements PIDOutput {
	//	global variables

	//  Declaring strings for the auton based on FMS data
	String autoSelected;
	final String L = "AutoLeft";
	final String R = "AutoRight";
	final String Default = "AutoDefault";
	String gameData;

	//  Declaring drivetrain Speed Controllers
	final SpeedController frontLeftMotor;
	final SpeedController frontRightMotor;
	final SpeedController backLeftMotor;
	final SpeedController backRightMotor;

	//  Declaring Speed Controller Groups
	final SpeedControllerGroup leftMotors;
	final SpeedControllerGroup rightMotors;


	//  Declaring Encoders for drivetrain motor control
	final Encoder rightEncoder = new Encoder(5, 4, 6);
	final Encoder leftEncoder = new Encoder(8, 7, 9);

	//  Declaring Xbox controllers for controlling robot
	final XboxController pilotController;
	final XboxController copilotController;

	//  Declaring Drivetrain for moving the robot
	final DifferentialDrive driveTrain;

	//	Declaring NavX gyro for PID controller
	AHRS ahrs;																			

	//	Declaring variables for the PID controller
	PIDController turnController;
	double rotateToAngleRate;

	/* The following PID Controller coefficients will need to be tuned */
	/* to match the dynamics of your drive system.  Note that the      */
	/* SmartDashboard in Test mode has support for helping you tune    */
	/* controllers by displaying a form where you can enter new P, I,  */
	/* and D constants and test the mechanism.                         */

	//	These are the default values -UNUSED-
	//	static final double kP = 2; //0.015;
	//	static final double kI = 5; //0.00
	//	static final double kD = 0.035;

	//	Rotational Constants for turning x degrees where x is kTargetAngleDegrees
	double kD = 0.105;//.035
	final double kF = 0.00;

	double kP = 0.0205;
	double kI = 0.00012;

	//	Constants for PID Controller for moving straight
	//	double kD = 0.255;//.035
	//	final double kF = 0.00;

	//	double kP = 0.0175;
	//	double kI = 0.0085;

	//	Declaring PID variables 
	double testSpd = .4;
	double newSpd = 0;
	static final double kToleranceDegrees = 1;    
	static final double kTargetAngleDegrees = 90;

	//  Declaring timer used in auto
	//	Timer autoTimer;

	/*	//  Declaring Ultrasonics used in auto
	Ultrasonic lUltra;
	Ultrasonic rUltra;
	 */

	//  Declaring Pixy camera used for vision
	PixyCrate myPixy;

	//	Declaring GRIP method
	GripPipeline CubeHunter;

	//	Declaring the USB Camera
	UsbCamera camera;

	//	Declaring the vision thread
	Thread m_visionThread;

	//	Declaring the mat for vision
	Mat mat;

	//	Declaring the Network Table for GRIP outputs
	NetworkTable gripOutputs;
	NetworkTableInstance gripInstance;
	NetworkTableEntry xGrip;
	NetworkTableEntry yGrip;
	NetworkTableEntry areaGrip;


	//	Declares array for storing Grip values
	double[] gripX;
	double[] defaultVal;

	//	Declaring variables for the Crate Grabber Arm
	CrateGrabber grabberArm;
	boolean armFlag;
	boolean raisedArm;
	double cubeIntakeSpeed;
	double cubeLaunchSpeed;

	//	Declares variables for auton driving - first flag is used to determine if it is the first time we have entered the method that case
	int autoCase;
	boolean firstFlag;
	double rDistance;
	double lDistance;

	//	Declares our Robot Climber
	RobotClimber climber;

	/*
	 * This is our robot's constructor.
	 */
	public Robot() {
		//  Instantiating Speed Controllers and assigned ports
		frontLeftMotor = new VictorSP(0);
		frontLeftMotor.setInverted(false);
		backLeftMotor = new VictorSP(1);
		backLeftMotor.setInverted(true);
		frontRightMotor = new VictorSP(2);
		frontRightMotor.setInverted(false);
		backRightMotor = new VictorSP(3);
		backRightMotor.setInverted(false);

		//  Instantiating Speed Controller Groups
		leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
		rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

		//  Instantiating Xbox Controllers
		pilotController = new XboxController (0);
		copilotController = new XboxController (1);

		//  Instantiating drivetrain
		driveTrain = new DifferentialDrive(leftMotors, rightMotors);

		//	NavX
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
//			 ************************************************************************/
			ahrs = new AHRS(SPI.Port.kMXP); 
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		//turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
		turnController = new PIDController(kP, kI, kD, kF, ahrs, this, 0.02);
		turnController.setInputRange(-180.0f,  180.0f);
		//turnController.setOutputRange(-1.0, 1.0);
		turnController.setOutputRange(-.75, .75);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		//	turnController.setPercentTolerance(5);
		turnController.setContinuous(true);
		turnController.disable();

		/* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		/* tuning of the Turn Controller's P, I and D coefficients.            */
		/* Typically, only the P value needs to be modified.                   */
		//	LiveWindow.addActuator("DriveSystem", "RotateController", turnController);        
		SmartDashboard.putNumber("KP", turnController.getP());
		SmartDashboard.putNumber("KI", turnController.getI());
		SmartDashboard.putNumber("KD", turnController.getD());
		SmartDashboard.putNumber("Speed", testSpd);

		/*//  Instantiating ultrasonics
		lUltra = new Ultrasonic(1,0);
		rUltra = new Ultrasonic(3,2);
		lUltra.setAutomaticMode(true);
		 */
		cubeLaunchSpeed = 0.4;
		cubeIntakeSpeed = 0.3;

		autoCase = 0;
		firstFlag = true;
		rDistance = 0;
		lDistance = 0;
		grabberArm = new CrateGrabber(4, 5, 0, 1, 2, 3, 4, 5);

		climber = new RobotClimber(6, 7, 6, 7);

	}

	@Override
	public void robotInit(){

		//lUltra.setEnabled(true);
		//rUltra.setEnabled(true);
		
		//  Instantiating pixy camera
		myPixy = new PixyCrate();

		//	Instantiates a USB camera must be commented out when no camera is present
		//camera = CameraServer.getInstance().startAutomaticCapture();

		//	Creates a Mat for storing images vision code
		mat = new Mat();

		//	Creates a GRIP pipeline for processing the images received from the USB Camera
		CubeHunter = new GripPipeline();

		//  Must be commented out if there is no camera
		//	Creates a thread for running the Camera
		//		m_visionThread = new Thread(() -> {
		//			//  Get the UsbCamera from CameraServer
		//			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		//
		//			//  Set the resolution
		//			camera.setResolution(640, 480);
		//
		//			//  Get a CvSink. This will capture Mats from the camera
		//			CvSink cvSink = CameraServer.getInstance().getVideo();
		//
		//			// Setup a CvSource. This will send images back to the Dashboard
		//			CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 640, 480);
		//
		//			//  This cannot be 'true'. The program will never exit if it is. This
		//			//  lets the robot stop this thread when restarting robot code or
		//			//  deploying.
		//			while (!Thread.interrupted()) {
		//				//  Tell the CvSink to grab a frame from the camera and put it
		//				//  in the source mat.  If there is an error notify the output.
		//				if (cvSink.grabFrame(mat) == 0) {
		//					//  Send the output the error.
		//					outputStream.notifyError(cvSink.getError());
		//					//  skip the rest of the current iteration
		//					continue;
		//				}
		//			}
		//
		//			// Give the output stream a new image to display
		//			outputStream.putFrame(mat);
		//
		//		});
		
		//  Sets up distance for pulse so getPulse is set in inches
		leftEncoder.setDistancePerPulse(0.0092);
		rightEncoder.setDistancePerPulse(0.0092);
	}

	public void autonomousInit(){
		ahrs.zeroYaw();

		//  Gets the string from the FMS that shows which side of the switch and scale
		//  This is mainly a failsafe if the string doesn't get to the robot
		//  This is the failsafe so the fatal error doesn't kill the robot
		try {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		catch (RuntimeException ex ) {
			DriverStation.reportError("Error retrieveing switch and scale data from FMS:  " + ex.getMessage(), true);
			autoSelected = Default;
		}
		
		//	Gets FMS data, chooses Auto case based on it
		if(gameData.charAt(0) == 'L') {
			//	Sets robot to use code to go left
			autoSelected = L;
			System.out.println("Left Auto Selected");
		}
		else if(gameData.charAt(0) == 'R') {
			//	Sets robot to use code to go right
			autoSelected = R;
			System.out.println("Right Auto Selected");
		}
		else {
			//	Sets robot to use default code
			autoSelected = Default;
			System.out.println("Default Auto Selected");
		}

	}

	public void autonomousPeriodic(){
		switch(autoSelected) {
		case L:
			//	Left auto Code here
			System.out.println("Left case");
			break;
		case R:
			//	Right auto code here
			System.out.println("Right case");
			break;
		case Default:
			//	Default auto code here
			System.out.println("Default case");
			break;
		}

		driveTrain.setSafetyEnabled(true);
		
		//  A case based test auton for driving in a square
		switch(autoCase){
		//  Turns 90 degrees counterclockwise
		case(0):
			RotateDrive(-90);
		break;
		//  Moves forward 10 inches at a velocity of 0.2
		case(1):
			StraightDrive(10, 0.2);
		break;
		//  Moves forward 10 inches at a velocity of 0.2
		case(2):
			StraightDrive(10, 0.2);
		break;
		//  Turns 90 degrees counterclockwise
		case(3):
			RotateDrive(-90);
		break;
		//  Moves forward 10 inches at a velocity of 0.2
		case(4):
			StraightDrive(10, 0.2);
		break;
		//  Turns 0 degrees
		case(5):
			RotateDrive(0);
		break;
		//  Moves forward 10 inches at a velocity of 0.2
		case(6):
			StraightDrive(10, 0.2);
		break;
		//  Turns 90 degrees clockwise
		case(7):
			RotateDrive(90);
		break;
		default:
		}
		
		//  Commented out for testing purposes
		//			Auton for testing vision
		//			A method that turns the robot to face the target
		//			myPixy.centerOnObject(driveTrain);
		//			Timer.delay(.07);
	
		//		Instantiating GRIP Network Tables
		gripOutputs = gripInstance.getTable("GRIP/myBlobsReport");
		xGrip = new NetworkTableEntry(gripInstance, 0);
		yGrip = new NetworkTableEntry(gripInstance, 1);
		areaGrip = new NetworkTableEntry(gripInstance, 2);
		defaultVal = new double[1];
		defaultVal[1] = -1;

		Timer.delay(0.05);		// wait for a motor update time
	}

	/**
	 * Method for driving straight in auton
	 * @param targetDistance The distance you want to travel (in inches)
	 * @param speed The speed that you want the robot to travel at (range is -1 to 1)
	 */
	public void StraightDrive(double targetDistance, double speed){

		//	Resets the encoders and the distance traveled the first time this enters
		if(firstFlag){
			leftEncoder.reset();
			rightEncoder.reset();

			rDistance = 0;
			lDistance = 0;

			System.out.println("resetting");

			// Sets the Setpoint so the robot travels straight
			turnController.setSetpoint(0);

			firstFlag = false;
		}

		//	Enables the turn controller if it is not already
		if (!turnController.isEnabled()) {
			rotateToAngleRate = 0;
			turnController.enable();
		}

		//	Gets the total distance from the encoders
		//	This encoder must be inverted so that we get proper values
		rDistance -= rightEncoder.getDistance();
		lDistance += leftEncoder.getDistance();

		//	Prints distance from encoders
		System.out.println(rDistance + "   " + lDistance);

		//	Gets rate of rotation from PID
		rotateToAngleRate = turnController.get();

		//	Stops robot if target distance was reached and moves to the next case
		if(targetDistance <= leftEncoder.getDistance() || targetDistance <= rightEncoder.getDistance()){
			driveTrain.arcadeDrive(0, 0, false);
			autoCase++;
			firstFlag = true;
		}

		//	Drives straight forward if target is not reached
		else{
			driveTrain.arcadeDrive(speed, rotateToAngleRate, false);
		}
	}

	/**
	 * Method for rotating into the target angle in auton
	 * @param targetAngle The angle we want to turn to (in degrees)
	 */
	public void RotateDrive(double targetAngle){
		//  If this is the first time entering this method, sets target angle
		if(firstFlag){
			turnController.setSetpoint(targetAngle);
			firstFlag = false;
		}

		//	 If the turn controller is not enabled, enable turn controller
		if (!turnController.isEnabled()) {
			//	rotateRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}

		//	Sets the speed the the robot rotates at from the PID
		rotateToAngleRate = turnController.get();

		//	Prints setpoint and rotation rate
		System.out.println(turnController.getSetpoint());
		System.out.println(rotateToAngleRate);

		//	If the PID has slowed down to a certain point, exit the case
		if(rotateToAngleRate < 0.0125 && rotateToAngleRate > -0.0125){
			//	If we have, stop and return true
			autoCase++;
			firstFlag = true;
		}

		//	Makes the robot turn to angle
		driveTrain.arcadeDrive(0, rotateToAngleRate, false);
	}



	public void teleopInit(){
		driveTrain.setSafetyEnabled(true);

		armFlag = false;
		raisedArm = false;
	}

	/**
	 *  The code that runs periodically while the robot is in teleop mode
	 */
	public void teleopPeriodic() {

		//	Drives robot based on joysticks - inverted because y output on sticks is also inverted
		driveTrain.tankDrive(-pilotController.getY(Hand.kLeft), -pilotController.getY(Hand.kRight), true);
		Timer.delay(0.1);

		//  If armFlag is false and the A button on the copilot controller is pressed, close the crate arm
		if(armFlag == false){
			if(copilotController.getAButton()){
				grabberArm.closeGrabber(true);
				armFlag = true;
			}
		}
		
		//  If armFlag is true and the A button on the copilot controller is pressed, open the crate arm
		else if(armFlag){
			if(copilotController.getAButton()){
				grabberArm.openGrabber(true);
				armFlag = false;
			}
		}

		//	Prints whether the arms should be open or closed where open is false and closed is true
		System.out.println(armFlag);
		
		/*	Commented out because there isn't currently a way to raise the arm
		 * if(raisedArm == false){
			if(copilotController.getBButton()){
				grabberArm.raiseArm();
			}
		}
		else if(raisedArm){
			if(copilotController.getBButton()){
				grabberArm.lowerArm();
			}
		}*/

		//	If there is not a cube in the grabber and Y button is pressed turn motors on to intake cube
		if(copilotController.getYButton()){
			if(grabberArm.detectCube() == false){
				grabberArm.cubeIntake(cubeIntakeSpeed);
			}
			//	If cube is detected stop intake motors
			else if(grabberArm.detectCube()){
				grabberArm.stopIntake();
			}
		}
		//	Launches cube when X button is pressed
		else if (copilotController.getXButton()){
			grabberArm.launchCube(cubeLaunchSpeed);
		}

		//	Controls for the climber based on copilot pressing the left trigger and right bumper
		climber.winchControl(copilotController.getTriggerAxis(Hand.kLeft), copilotController.getBumper(Hand.kRight));

		//	If the Y stick is pressed up, extend the climber. If it is pulled back, retract the climber
		//  The comparison is inverted due to the Y-stick naturally being inverted
		if(copilotController.getY(Hand.kLeft) > -0.9){
			climber.extendSolenoid();
		}
		else if(copilotController.getY(Hand.kLeft) < 0.9){
			climber.retractSolenoid();
		}
	}

	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX MXP yaw angle input and PID Coefficients.    */
	public void pidWrite(double output) {
		//rotateToAngleRate = output;
	}


	public void testInit(){
		ahrs.zeroYaw();
		/*
		if (!turnController.isEnabled()) {
			turnController.setSetpoint(kTargetAngleDegrees);
			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}
		 */
		gripX = new double[xGrip.getDoubleArray(defaultVal).length];
		for(int i = 0; i < xGrip.getDoubleArray(defaultVal).length; i++){
			gripX[i] = -1;
		}
		turnController.reset();
			//System.out.println(SmartDashboard.getNumber("Speed", -10));
	}


		public void testPeriodic(){
			if (!turnController.isEnabled()) {
				turnController.setSetpoint(kTargetAngleDegrees);
				rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
				turnController.enable();
			}

			//  Runs our GRIP pipeline given the webcam input
			//  Mat given by vision thread
			CubeHunter.process(mat);
			
			//  Used to edit PID constants in the LiveWindow Dashboard
			//		Sendable sendable = null;
			//		LiveWindow.add(sendable);
			//		LiveWindow.addActuator(turnController, m_P, turnController.getP());
			//		LiveWindow.add(turnController.getSubsystem());
			//		LiveWindow.addChild(turnController, turnController.getD());
			
			
			System.out.println(turnController.onTarget());
			rotateToAngleRate = turnController.get();
			System.out.println((double)rotateToAngleRate);
			driveTrain.arcadeDrive(0, rotateToAngleRate, false);
			Timer.delay(.005);

			gripX = xGrip.getDoubleArray(gripX);

			//	TODO Needs to be fixed, will not print values
			for (double e:gripX){
				System.out.print(e + "\t");
			}
			System.out.println();
		}
	}
