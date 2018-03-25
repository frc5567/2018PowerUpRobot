package org.usfirst.frc.team5567.robot;

import org.usfirst.frc.team5567.robot.Grabber.AngleState;
import org.usfirst.frc.team5567.robot.Grabber.ArmState;
import org.usfirst.frc.team5567.robot.Grabber.MotorState;
import org.usfirst.frc.team5567.robot.RobotClimber.ClimbState;
import org.usfirst.frc.team5567.robot.AutoDrive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//  This is the main robot that we use in competition.
public class Robot extends IterativeRobot implements PIDOutput {
	
	AutoDrive autoDrive;
	
//	Declares Enum for changing the control method for grabber
	public enum GrabberMode {
		kAutomatic(0),
		kManual(1);

		private int grabberMode;

		GrabberMode(int grabberMode) {
			this.setGrabberMode(grabberMode);
		}

		/**
		 * @return the grabberMode
		 */
		public int getGrabberMode() {
			return grabberMode;
		}

		/**
		 * @param grabberMode the grabberMode to set
		 */
		public void setGrabberMode(int grabberMode) {
			this.grabberMode = grabberMode;
		}
	}
	
	//	global variables
	
	//	Temporary flag to tell us what state we are in until I figure out enums
	boolean manualGrabberControl = true;
	
	//	Declaring DigitalInput for the IR break beam sensor used to detect cubes
	final DigitalInput boxTrip;
	
	//	Sets the type of grabber where true is motor and false is Solenoid
	private static final boolean ARM_TYPE = true;
	//  Declaring strings for the auton based on FMS data
	String fmsAutoSelected;
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
	final Encoder rightEncoder = new Encoder(5, 4, true, EncodingType.k1X);
	final Encoder leftEncoder = new Encoder(8, 7, false, EncodingType.k1X);

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
	PIDController straightController;

	//	Declares integer for counting zeroes while turning
	int rotateCount;

	//	Declares Threshold for counting zeroes while turning in auto
	double rotateThreshold = 0.2;

	//	Rotate PID Constants for testing 3/1/2018
	double kDRotate = 0.015;
	double kFRotate = 0.00;
	double kPRotate = 0.0078;
	double kIRotate = 0.0001;

	//	Comp values
	/*//	Rotational Constants for turning x degrees where x is kTargetAngleDegrees
	double kDRotate = 0.001;
	final double kFRotate = 0.00;

	double kPRotate = 0.0078;
	double kIRotate = 0.0001;*/

	//	Constants for PID Controller for moving straight
	double kDStraight = 0.105;
	final double kFStraight = 0.00;

	double kPStraight = 0.0175;
	double kIStraight = 0.000212;

	//	Declaring PID variables 
	double testSpd = .4;
	double newSpd = 0;

	//	Tolerance in degrees for the PID controller in the different driving modes (straight and rotating)
	static final double kToleranceRotate = 3;
	static final double kToleranceStraight = 0.1;

	static final double kTargetAngleDegrees = 90;

	//	Declaring the USB Camera
	UsbCamera camera;
	//	UsbCamera cameraTwo;

	//	Declaring variables for the Crate Grabber Arm
	Grabber grabberArm;

	//	Variables that define motor speeds in auton for the arm
	double cubeIntakeSpeed;
	double cubeLaunchSpeed;

	//	Declares variables for auton driving - first flag is used to determine if it is the first time we have entered the method that case
	static int autoCase;
	boolean firstFlag;
	double rDistance;
	double lDistance;

	//	Declares our Robot Climber
	RobotClimber climber;


	//	Declares sendable chooser variables
	private static final String kRightPosition = "Right Position";
	private static final String kLeftPosition = "Left Auton";
	private static final String kNoMoveAuton = "No Move Auton";
	private static final String kStraightAuton = "Straight Auton";
	private String m_dashboardAutoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
 
	//	Instantiates a value that should equal 90 degrees on the encoder that we will use to break a while loop
	final int kInitAngle = -130;

	//	Declaring a PixyCamera for auton tracking
	PixyCrate pixyCamera;

	DigitalInput breakBeam;

	double matchTimer = 0.0;
	/*
	 * This is our robot's constructor.
	 */
	public Robot() {
		frontLeftMotor = new Spark(0);
		backLeftMotor = new Spark(1);
		frontRightMotor = new Spark(2);
		backRightMotor = new Spark(3);
		//	Set on test robot, false for comp bot
		frontRightMotor.setInverted(true);
		// Edited for Spark MotorController  Unsure of Comp robot
		backLeftMotor.setInverted(true);

		//	Instantiating IR break beam sensor 
		boxTrip = new DigitalInput(3);
		
		//	Comp motor Controllers
		/*//  Instantiating Speed Controllers and assigned ports
		frontLeftMotor = new VictorSP(0);
		backLeftMotor = new VictorSP(1);
		frontRightMotor = new VictorSP(2);
		//	Changed to true for test robot, false for comp bot
		frontRightMotor.setInverted(true);
		backRightMotor = new VictorSP(3);*/

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

		//	Sets up the rotation PID
		turnController = new PIDController(kPRotate, kIRotate, kDRotate, kFRotate, ahrs, this, 0.02);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-.4, .4);
		turnController.setAbsoluteTolerance(kToleranceRotate);
		turnController.setContinuous(true);
		turnController.disable();

		//	Sets up the straight PID
		straightController = new PIDController(kPStraight, kIStraight, kDStraight, kFStraight, ahrs, this, 0.02);
		straightController.setInputRange(-180.0f,  180.0f);
		straightController.setOutputRange(-.8, .8);
		straightController.setAbsoluteTolerance(kToleranceStraight);
		straightController.setContinuous(true);
		straightController.disable();


		/* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		/* tuning of the Turn Controller's P, I and D coefficients.            */
		/* Typically, only the P value needs to be modified.                   */
		SmartDashboard.putNumber("KP", turnController.getP());
		SmartDashboard.putNumber("KI", turnController.getI());
		SmartDashboard.putNumber("KD", turnController.getD());
		SmartDashboard.putNumber("Speed", testSpd);
		
		SmartDashboard.putBoolean("Pilot Control", manualGrabberControl);

		cubeLaunchSpeed = 0.8;
		cubeIntakeSpeed = 0.7;

		//	Instantiates variables for the auton methods
		autoCase = 0;
		firstFlag = true;
		rDistance = 0;
		lDistance = 0;

		//	Instantiates our Grabber and Climber
		if(ARM_TYPE){
			//grabberArm = new CrateGrabberMotor(4, 5, 0, 1, 9);
			grabberArm = new CrateGrabberMotor(4, 5, 6, 7, 9);
		}
		else if(!ARM_TYPE){
			//grabberArm = new CrateGrabberSol(4, 5, 0, 1, 2, 3);	
		}

		//climber = new RobotClimber(6, 7, 6, 7);

		// COMP ROBOT
		//climber = new RobotClimber(6,7,0,1);

		// Test Robot
		climber = new RobotClimber(6,7,2,1);

		breakBeam = new DigitalInput(10);
	}

	@Override
	public void robotInit(){

		//	Sets up the camera stream
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		//	cameraTwo = CameraServer.getInstance().startAutomaticCapture(1);
		camera.setResolution(160, 120);
		camera.setFPS(1);
		//	cameraTwo.setResolution(320, 240);
		//	cameraTwo.setFPS(30);

		//  Sets up distance for pulse so getDistance is set in inches
		leftEncoder.setDistancePerPulse(0.0092);
		rightEncoder.setDistancePerPulse(0.0092);

		//	Adds options to the sendable chooser
		m_chooser.addDefault("No Move Auton", kNoMoveAuton);
		m_chooser.addObject("Right Position", kRightPosition);
		m_chooser.addObject("Left Position", kLeftPosition);
		m_chooser.addObject("Straight Auton", kStraightAuton);

		SmartDashboard.putData("Auto choices", m_chooser);
		climber.setClimbSolenoid(ClimbState.kRetract);
		grabberArm.setGrabberArm(ArmState.kClosed);

		Timer.delay(1);
		grabberArm.armEncoder.reset();
		System.out.println("Reset the encoder on the CrateGrabber");
		Timer.delay(2);

		//	Instantiates pixy camera for tracking auton
		pixyCamera = new PixyCrate();

	}

	public void autonomousInit(){

		//	Zeros our gyro
		ahrs.zeroYaw();

		//	Sets the encoder distances to zero
		lDistance = 0;
		rDistance = 0;

		//	Activates safety for the drive train
		driveTrain.setSafetyEnabled(true);

		//	Zeros the autocase
		autoCase = 0;

		m_dashboardAutoSelected = kStraightAuton;
		System.out.println("Auto selected: " + m_dashboardAutoSelected);

		//	Sets initial position to the raised position
		grabberArm.armEncInit = grabberArm.armEncoder.getRaw();

		//	Sets raised position to an offset of the init position
		grabberArm.armEncRaised = grabberArm.armEncInit + 5;
	}

	public void autonomousPeriodic(){

		System.out.println(autoCase);
		switch (m_dashboardAutoSelected) {
		case kStraightAuton:
			switch(fmsAutoSelected) {
			case L:	//	TODO: This is broken, so it should be fixed at some point (soon)
		//	The 'fix' I put in is changing the distances and angle from straight ahead to match the right side 
				switch(autoCase){
				//  Drives straight
				case(0):
					autoDrive.StraightDriveAngle(56, 0.8, -60);
				System.out.println(leftEncoder.getDistance());
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					System.out.println(leftEncoder.getDistance());
				autoDrive.StraightDriveAngle(25, 0.3, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					autoDrive.StraightDriveAngle(28, 0.3, 0);
				System.out.println(leftEncoder.getDistance());
				System.out.println(Timer.getFPGATimestamp());
				if(matchTimer == 0){
					matchTimer = Timer.getFPGATimestamp();
				}
				if((Timer.getFPGATimestamp() - matchTimer) > 1){
					autoCase++;
					matchTimer = 0;
				}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(3):
					autoDrive.StraightDriveAngle(28, 0.3, 0);
				System.out.println(leftEncoder.getDistance());
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				System.out.println(Timer.getFPGATimestamp());
				if(matchTimer == 0){
					matchTimer = Timer.getFPGATimestamp();
				}
				if((Timer.getFPGATimestamp() - matchTimer) > 1){
					autoCase++;
				}
				break;
				default:
					grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeIntakeSpeed);
					break;
				}
				break;
			case R:
				switch(autoCase){
				//  Drives straight
				case(0):
					autoDrive.StraightDriveAngle(66, 0.8, 15);
				System.out.println(leftEncoder.getDistance());
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					System.out.println(leftEncoder.getDistance());
				autoDrive.StraightDriveAngle(25, 0.3, 15);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					autoDrive.StraightDriveAngle(28, 0.3, 15);
				System.out.println(leftEncoder.getDistance());
				System.out.println(Timer.getFPGATimestamp());
				if(matchTimer == 0){
					matchTimer = Timer.getFPGATimestamp();
				}
				if((Timer.getFPGATimestamp() - matchTimer) > 1){
					autoCase++;
					matchTimer = 0;
				}
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(3):
					autoDrive.StraightDriveAngle(28, 0.3, 15);
				System.out.println(leftEncoder.getDistance());
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				System.out.println(Timer.getFPGATimestamp());
				if(matchTimer == 0){
					matchTimer = Timer.getFPGATimestamp();
				}
				if((Timer.getFPGATimestamp() - matchTimer) > 1){
					autoCase++;
				}
				break;
				default:
					grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeIntakeSpeed);
					break;
				}
			}
			break;
		case kRightPosition:
			switch(fmsAutoSelected) {
			case L:
				//	Left auto Code here
				switch(autoCase){
				//  Drives straight
				case(0):
					autoDrive.StraightDriveAngle(204, 0.8, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					autoDrive.RotateDrive(-90);
				if(ahrs.getAngle() > -95 && ahrs.getAngle() < -85) {
					autoCase++;
					firstFlag = true;
				}
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					autoDrive.StraightDriveAngle(194, 0.8, -90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(3):
					autoDrive.RotateDrive(135);
				if(ahrs.getAngle() > -230 && ahrs.getAngle() < -220) {
					autoCase++;
					firstFlag = true;
				}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(4):
					autoDrive.StraightDriveAngle(30, 0.8, 135);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(5):

					grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				Timer.delay(1.5);
				autoCase++;
				break;
				/*case(3):
					RotateDrive(-180);
				if(ahrs.getAngle() > -185 && ahrs.getAngle() < -175) {
					autoCase++;
					firstFlag = true;
				}
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(4):
					StraightDriveAngle(50, 0.8, -180);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(5):
					RotateDrive(90);
				if(ahrs.getAngle() > -275 && ahrs.getAngle() < -265) {
					autoCase++;
					firstFlag = true;
				}
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(6):
					StraightDriveAngle(10, 0.8, 90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(7):
					grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				Timer.delay(1.5);
				autoCase++;
				break;*/
				default:
					break;
				}
				break;
			case R:
				//	Right auto code here
				System.out.println("Right case");
				System.out.println("R-Pos");
				System.out.println("Left Encoder Val   " + leftEncoder.getDistance());

				//				Left auto Code here
				switch(autoCase){
				//  Drives straight
				case(0):
					autoDrive.StraightDriveAngle(/*168*/ 130, 0.6, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					autoDrive.RotateDrive(-90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				if(ahrs.getAngle() > -95 && ahrs.getAngle() < -85) {
					autoCase++;
					firstFlag = true;
				}
				break;
				case(2):
					autoDrive.StraightDriveAngle(10, 0.4, -90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(3):
					autoDrive.StraightDriveAngle(25, 0.25, -90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				Timer.delay(1.5);
				autoCase++;
				break;
				case(4):	//	Cube delivered, drive away
					autoDrive.StraightDriveAngle(-40, -.5, -30);					
				break;
				case(5):	//	Drive towards the center
					autoDrive.StraightDriveAngle(40, .5, -120);
				break;
				default:
					break;
				}
				break;

			case Default:
				//	Default auto code here
				System.out.println("Default case");
				break;
			}
			break;

		case kLeftPosition:
			switch(fmsAutoSelected) {
			case L:
				//	Left auto Code here
				switch(autoCase){
				//  Drives straight
				case(0):
					autoDrive.StraightDriveAngle(130/*210*/, .8, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					autoDrive.RotateDrive(90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					autoDrive.StraightDriveAngle(25, 0.6, 90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(3):
					autoDrive.StraightDriveAngle(25, 0.25, 90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				Timer.delay(1.5);
				autoCase++;
				break;
				default:
					break;
				}
				break;
			case R:
				//	Right auto code here
				System.out.println("Right case");

				//				Left auto Code here
				switch(autoCase){
				case(0):
					autoDrive.StraightDriveAngle(198, 0.8, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					autoDrive.RotateDrive(90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					autoDrive.StraightDriveAngle(219, 0.8, 90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(3):
					autoDrive.RotateDrive(180);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(4):
					autoDrive.StraightDriveAngle(52, 0.8, 180);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(5):
					autoDrive.RotateDrive(-90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(6):
					autoDrive.StraightDriveAngle(4, 0.8, -90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(7):
					grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				Timer.delay(1.5);
				autoCase++;
				break;
				default:
					break;
				}
				break;

			case Default:
				//	Default auto code here
				System.out.println("Default case");
				break;
			}
			break;

		case kNoMoveAuton:
		default:
			// Put default auto code here
			break;
		}

		Timer.delay(0.05);		// wait for a motor update time
	}



	public void teleopInit(){
		driveTrain.setSafetyEnabled(true);

		//	Resets the encoders
		rightEncoder.reset();
		leftEncoder.reset();

		rDistance = 0;
		lDistance = 0;
	}

	/**
	 *  The code that runs periodically while the robot is in teleop mode
	 */
	public void teleopPeriodic() {
		///*
		//	Drives robot based on video game style controls, rTrigger is forward, lTrigger is reverse, left stick is turning
		driveTrain.arcadeDrive(pilotController.getTriggerAxis(Hand.kRight)-pilotController.getTriggerAxis(Hand.kLeft), pilotController.getX(Hand.kLeft), true);
		//		Timer.delay(0.05);

		//	Sets rdistance to the right encoder value
		rDistance = rightEncoder.getDistance();
		lDistance = leftEncoder.getDistance();

		System.out.println(leftEncoder.getRate());
		//	Prints the encoder data.
		//		System.out.println("R:[" +rDistance+ "][" +rightEncoder.getRaw()+ "] L:[" +lDistance+ "][" +leftEncoder.getRaw()+ "]");


		//	Controls grabber based on selected mode	//TODO
		if(copilotController.getStartButtonReleased()){
			manualGrabberControl = true;
			SmartDashboard.putBoolean("Pilot Control", manualGrabberControl);
		}
		else if(copilotController.getBackButtonReleased()){
			manualGrabberControl = false;
			SmartDashboard.putBoolean("Pilot Control", manualGrabberControl);
		}
		
		if(manualGrabberControl){
			grabberControl(GrabberMode.kManual);
		}
		else{
			grabberControl(GrabberMode.kAutomatic);
		}
		
		//			TODO More commented out crate arm closed and open
		if(copilotController.getBButtonReleased()){
			grabberArm.setGrabberArm(ArmState.kOpen);
		}
		
		else if(boxTrip.get() == false) {
			grabberArm.setGrabberArm(ArmState.kClosed);
		}
		
		else if(copilotController.getAButtonReleased()){
			grabberArm.setGrabberArm(ArmState.kClosed);
		}

		//	Raises the arm if the right bumper is pressed
		if(copilotController.getBumper(Hand.kRight)){
			grabberArm.setAngleArm(AngleState.kRaised, 0.7);
		}

		//	Lowers arm if the left bumper is pressed
		if(copilotController.getBumper(Hand.kLeft)){
			grabberArm.setAngleArm(AngleState.kLowered, 0.4);

		}

		//	TODO Cube grabber in Teleop 
		//	If there is not a cube in the grabber and X button is pressed turn motors on to intake cube
		if(Math.abs(copilotController.getTriggerAxis(Hand.kRight))>0.1){
			//			if(grabberArm.detectCube() == false){
			grabberArm.setMotorArm(MotorState.kIntake, copilotController.getTriggerAxis(Hand.kRight), cubeLaunchSpeed);
			//			}
			//	If cube is detected stop intake motors
			//			else if(grabberArm.detectCube()){
			//				grabberArm.stopIntake();
			//			}
		}
		//	Launches cube when Y button is pressed
		else if (Math.abs(copilotController.getTriggerAxis(Hand.kLeft))>0.1){
			grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, copilotController.getTriggerAxis(Hand.kLeft));
		}

		//	Controls for the climber based on copilot pressing the left trigger and right bumper 
		if((copilotController.getXButton())){
			//TODO: update winch speeds in RobotClimber
			climber.winchControl(-.3, false);
		}

		if(copilotController.getYButton()){
			climber.winchControl(-.7, false);
		}

		//		climber.winchControl(copilotController.getTriggerAxis(Hand.kLeft), false);//copilotController.getBumper(Hand.kRight));
		//System.out.println(copilotController.getTriggerAxis(Hand.kLeft));
		//	If the Y stick is pressed up, extend the climber. If it is pulled back, retract the climber
		//  The comparison is inverted due to the Y-stick naturally being inverted
		if(copilotController.getY(Hand.kLeft) < -0.8){
			climber.setClimbSolenoid(ClimbState.kExtend);
			//System.out.println("here");
		}
		else if(copilotController.getY(Hand.kLeft) > 0.8){
			climber.setClimbSolenoid(ClimbState.kRetract);
		}		
	}
	
	public void grabberControl(GrabberMode grabberMode){
		switch(grabberMode){
		case kAutomatic:
			//	Sendable chooser stuff here
			if(!breakBeam.get()){
				grabberArm.setGrabberArm(ArmState.kClosed);
				//	grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeLaunchSpeed);
			}
			else{
				//	grabberArm.setMotorArm(MotorState.kIntake, cubeIntakeSpeed, cubeLaunchSpeed);
				grabberArm.setGrabberArm(ArmState.kOpen);
			}
			/*if(!breakBeam2.get()){
				manualGrabberControl = true;
				grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeLaunchSpeed);
			}
			else{
				grabberArm.setMotorArm(MotorState.kIntake, cubeIntakeSpeed, cubeLaunchSpeed);
			}*/
			
			
			break;
		case kManual: 
			//	Intakes cube when  right trigger is pressed
			if(Math.abs(copilotController.getTriggerAxis(Hand.kRight))>0.1){
				grabberArm.setMotorArm(MotorState.kIntake, copilotController.getTriggerAxis(Hand.kRight), cubeLaunchSpeed);
			}
			//	Launches cube when left trigger is pressed
			else if (Math.abs(copilotController.getTriggerAxis(Hand.kLeft))>0.1){
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, copilotController.getTriggerAxis(Hand.kLeft));
			}
			//	Closes arms when A button is pressed
			if(copilotController.getAButtonReleased()){
				grabberArm.setGrabberArm(ArmState.kClosed);
			}
			//  Opens arms when B button is pressed
			if(copilotController.getBButtonReleased()){
				grabberArm.setGrabberArm(ArmState.kOpen);
			}
			
			break;
		default:
			break;
		
		}
	}	
	
	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX MXP yaw angle input and PID Coefficients.    */
	//	This method is required but unused
	public void pidWrite(double output) {
		//rotateToAngleRate = output;
	}


	public void testInit(){
		grabberArm.setGrabberArm(ArmState.kOpen);
		//grabberArm.setMotorArm(MotorState.kIntake, cubeIntakeSpeed, cubeLaunchSpeed);
	}


	public void testPeriodic(){

		/*if(copilotController.getAButton()){
			grabberArm.setGrabberArm(ArmState.kClosed);;
		}
		else{
			grabberArm.setGrabberArm(ArmState.kOpen);;
		}
		if(copilotController.getBButton()){
			grabberArm.dSolLeft.set(Value.kOff);
		}

		System.out.println(grabberArm.armSwitch.get());
		System.out.println(grabberArm.armEncoder.getRaw());*/
		System.out.println(breakBeam.get());
		/*if(!breakBeam.get()){
			grabberArm.setGrabberArm(ArmState.kClosed);
			grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeLaunchSpeed);
		}
		else{
			grabberArm.setMotorArm(MotorState.kIntake, cubeIntakeSpeed, cubeLaunchSpeed);
			grabberArm.setGrabberArm(ArmState.kOpen);
		}*/
		//pixyCamera.centerOnObject(driveTrain);
	}
}
