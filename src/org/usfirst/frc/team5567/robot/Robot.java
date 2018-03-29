package org.usfirst.frc.team5567.robot;

import org.usfirst.frc.team5567.robot.Grabber.AngleState;
import org.usfirst.frc.team5567.robot.Grabber.ArmState;
import org.usfirst.frc.team5567.robot.Grabber.MotorState;
import org.usfirst.frc.team5567.robot.Grabber.IntakeMode;
import org.usfirst.frc.team5567.robot.RobotClimber.ClimbState;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.Spark;
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
	// global variables

	double desiredSpeed;
	final double kMaxDeltaSpeed = 0.1;
	double currentSpeed;

	double desiredRotate;
	double currentRotate;

	// Sets the type of grabber where true is motor and false is Solonoid
	private static final boolean ARM_TYPE = true;
	// Declaring strings for the auton based on FMS data
	String fmsAutoSelected;
	final String L = "AutoLeft";
	final String R = "AutoRight";
	final String Default = "AutoDefault";
	String gameData;

	private boolean manualGrabberControl = false;

	// Declaring drivetrain Speed Controllers
	final SpeedController frontLeftMotor;
	final SpeedController frontRightMotor;
	final SpeedController backLeftMotor;
	final SpeedController backRightMotor;

	// Declaring Speed Controller Groups
	final SpeedControllerGroup leftMotors;
	final SpeedControllerGroup rightMotors;

	//TODO Bookmark for the encoders
	// Declaring Encoders for drivetrain motor control
	final Encoder rightEncoder = new Encoder(5, 4, false, EncodingType.k1X);
	final Encoder leftEncoder = new Encoder(8, 7, true, EncodingType.k1X);
	// Declaring Xbox controllers for controlling robot
	final XboxController pilotController;
	final XboxController copilotController;

	// Declaring Drivetrain for moving the robot
	final DifferentialDrive driveTrain;

	// Declaring NavX gyro for PID controller
	AHRS ahrs;

	// Declaring variables for the PID controller
	PIDController turnController;
	double rotateToAngleRate;
	PIDController straightController;

	// Declares integer for counting zeroes while turning
	int rotateCount;

	// Declares Threshold for counting zeroes while turning in auto
	double rotateThreshold = 0.2;

	// Rotate PID Constants for testing 3/1/2018
	double kDRotate = 0.015;
	double kFRotate = 0.00;
	double kPRotate = 0.0078;
	double kIRotate = 0.0001;

	// Comp values
	/*
	 * // Rotational Constants for turning x degrees where x is
	 * kTargetAngleDegrees double kDRotate = 0.001; final double kFRotate =
	 * 0.00;
	 * 
	 * double kPRotate = 0.0078; double kIRotate = 0.0001;
	 */

	// Constants for PID Controller for moving straight
	double kDStraight = 0.105;
	final double kFStraight = 0.00;

	double kPStraight = 0.0175;
	double kIStraight = 0.000212;

	// Declaring PID variables
	double testSpd = .4;
	double newSpd = 0;

	// Tolerance in degrees for the PID controller in the different driving
	// modes (straight and rotating)
	static final double kToleranceRotate = 3;
	static final double kToleranceStraight = 0.1;

	static final double kTargetAngleDegrees = 90;

	// Declaring the USB Camera
	UsbCamera camera;
	// UsbCamera cameraTwo;

	// Declaring variables for the Crate Grabber Arm
	Grabber grabberArm;

	// Variables that define motor speeds in auton for the arm
	double cubeIntakeSpeed;
	double cubeLaunchSpeed;

	// Declares variables for auton driving - first flag is used to determine if
	// it is the first time we have entered the method that case
	static int autoCase;
	boolean firstFlag;
	double rDistance;
	double lDistance;

	// Declares our Robot Climber
	RobotClimber climber;

	// Declares sendable chooser variables
	private static final String kRightPosition = "Right Auton";
	private static final String kLeftPosition = "Left Auton";
	private static final String kStraightLineAuton = "Straight Line Auton";
	private static final String kCenterAuton = "Center Auton";

	// Rotation based on gyro uses this speed. See rotateDrive(double)
	private static final double kRotateSpeed = 0.28;

	private String m_dashboardAutoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	// Instantiates a value that should equal 90 degrees on the encoder that we
	// will use to break a while loop
	//	This shouldn't be used, leaving it for now
	//final int kInitAngle = -130;

	// Declaring a PixyCamera for auton tracking
	PixyCrate pixyCamera;

	//	DigitalInput breakBeam;

	double matchTimer = 0.0;
	double fieldMatchTime;

	Timer testTimer;

	int testCase;
	/*
	 * This is our robot's constructor.
	 */

	public Robot() {
		frontLeftMotor = new VictorSP(0);
		backLeftMotor = new VictorSP(1);
		frontRightMotor = new VictorSP(2);
		backRightMotor = new VictorSP(3);
		// Set on test robot, false for comp bot
		//	frontRightMotor.setInverted(true);
		// Editted for Spark MotorController Unsure of Comp robot
		//	backLeftMotor.setInverted(true);

		// Instantiating Speed Controller Groups
		leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
		rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);

		// Instantiating Xbox Controllers
		pilotController = new XboxController(0);
		copilotController = new XboxController(1);

		// Instantiating drivetrain
		driveTrain = new DifferentialDrive(leftMotors, rightMotors);

		// NavX
		try {
			/***********************************************************************
			 * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART)
			 * and USB. - See
			 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and
			 * USB. - See
			 * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported. //
			 ************************************************************************/
			ahrs = new AHRS(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		// Sets up the rotation PID
		turnController = new PIDController(kPRotate, kIRotate, kDRotate, kFRotate, ahrs, this, 0.02);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-.7, .7);
		turnController.setAbsoluteTolerance(kToleranceRotate);
		turnController.setContinuous(true);
		turnController.disable();

		// Sets up the straight PID
		straightController = new PIDController(kPStraight, kIStraight, kDStraight, kFStraight, ahrs, this, 0.02);
		straightController.setInputRange(-180.0f, 180.0f);
		straightController.setOutputRange(-.8, .8);
		straightController.setAbsoluteTolerance(kToleranceStraight);
		straightController.setContinuous(true);
		straightController.disable();

		/* Add the PID Controller to the Test-mode dashboard, allowing manual */
		/* tuning of the Turn Controller's P, I and D coefficients. */
		/* Typically, only the P value needs to be modified. */
		SmartDashboard.putNumber("KP", turnController.getP());
		SmartDashboard.putNumber("KI", turnController.getI());
		SmartDashboard.putNumber("KD", turnController.getD());
		SmartDashboard.putNumber("Speed", testSpd);

		cubeLaunchSpeed = 1.0;
		cubeIntakeSpeed = 0.7;

		// Instantiates variables for the auton methods
		autoCase = 0;
		firstFlag = true;
		rDistance = 0;
		lDistance = 0;

		// Instantiates our Grabber and Climber
		if (ARM_TYPE) {
			// grabberArm = new CrateGrabberMotor(4, 5, 0, 1, 9);
			grabberArm = new CrateGrabberMotor(4, 5, 7, 6, 9);
		} else if (!ARM_TYPE) {
			// grabberArm = new CrateGrabberSol(4, 5, 0, 1, 2, 3);
		}

		// climber = new RobotClimber(6, 7, 6, 7);

		// COMP ROBOT
		// climber = new RobotClimber(6,7,0,1);

		// Test Robot
		climber = new RobotClimber(6, 7, 0, 1);


	}

	@Override
	public void robotInit() {

		desiredSpeed = 0;
		currentSpeed= 0;
		desiredRotate = 0;
		currentRotate = 0;

		// Sets up the camera stream
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		// cameraTwo = CameraServer.getInstance().startAutomaticCapture(1);
		camera.setResolution(160, 120);
		camera.setFPS(1);
		// cameraTwo.setResolution(320, 240);
		// cameraTwo.setFPS(30);

		// Sets up distance for pulse so getDistance is set in inches
		rightEncoder.setDistancePerPulse(0.0092);

		// Adds options to the sendable chooser
		m_chooser.addDefault("No Move Auton", kStraightLineAuton);
		m_chooser.addObject("Right Position", kRightPosition);
		m_chooser.addObject("Left Position", kLeftPosition);
		m_chooser.addObject("Center Position", kCenterAuton);

		SmartDashboard.putBoolean("Pilot Control", manualGrabberControl);
		SmartDashboard.putData("Auto choices", m_chooser);
		climber.setClimbSolenoid(ClimbState.kRetract);
		grabberArm.setGrabberArm(ArmState.kClosed);

		Timer.delay(1);
		grabberArm.armEncoder.reset();
		System.out.println("Reset the encoder on the CrateGrabber");
		Timer.delay(2);

		// Instantiates pixy camera for tracking auton
		pixyCamera = new PixyCrate();

		grabberArm.setGrabberArm(ArmState.kClosed);
	}

	public void autonomousInit() {

		this.manualGrabberControl = true;
		// Zeros our gyro
		ahrs.zeroYaw();

		// Sets the encoder distances to zero
		lDistance = 0;
		rDistance = 0;

		// Gets the string from the FMS that shows which side of the switch and
		// scale
		// This is mainly a failsafe if the string doesn't get to the robot
		// This is the failsafe so the fatal error doesn't kill the robot
		try {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error retrieveing switch and scale data from FMS:  " + ex.getMessage(), true);
			fmsAutoSelected = Default;
		}

		// Activates safety for the drive train
		driveTrain.setSafetyEnabled(true);

		// Zeros the autocase
		autoCase = 0;

		// Gets FMS data, chooses Auto case based on it
		if (gameData.charAt(0) == 'L') {
			// Sets robot to use code to go left
			fmsAutoSelected = L;
			System.out.println("Left Auto Selected");
		} else if (gameData.charAt(0) == 'R') {
			// Sets robot to use code to go right
			fmsAutoSelected = R;
			System.out.println("Right Auto Selected");
		} else {
			// Sets robot to use default code
			fmsAutoSelected = Default;
			System.out.println("Default Auto Selected");
		}
		m_dashboardAutoSelected = m_chooser.getSelected();
		//		System.out.println("Auto selected: " + m_dashboardAutoSelected);

		// Sets initial position to the raised position
		grabberArm.armEncInit = grabberArm.armEncoder.getRaw();

		// Sets raised position to an offset of the init position
		grabberArm.armEncRaised = grabberArm.armEncInit + 80;
	}

	public void autonomousPeriodic() {
		System.out.println(autoCase);
		//		System.out.println("R Encoder   " + rightEncoder.getDistance() + "   R distance   " + rDistance);
		switch (m_dashboardAutoSelected) {
		case kCenterAuton:
			switch (fmsAutoSelected) {
			case L: // TODO: When this left auton was last tested, the initial path was
				// not correct and we did not successfully place the cube. 
				// Need to test/tune prior to use
				switch (autoCase) {
				case (0):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(10, .7, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .3) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				//					straightDriveAngle(10, 0.4, 0, true);
				//				System.out.println(rightEncoder.getDistance());
				//				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				//				grabberArm.setGrabberArm(ArmState.kClosed);
				break;
				case (1):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}

				if (rotateDrive(-60) || ((Timer.getFPGATimestamp() - matchTimer) > 2)) {
					autoCase++;
					firstFlag = true;
					matchTimer = 0;
				}
				//					
				//					if (rotateDrive(-45)) {
				//						autoCase++;
				//					}
				break;
				case (2):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(10, .8, -60, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .8) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				//					System.out.println(rightEncoder.getDistance());
				//				straightDriveAngle(65, 0.8, -60, true);
				//				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case (3):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}

				if (rotateDrive(0) || ((Timer.getFPGATimestamp() - matchTimer) > 2)) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				//					System.out.println(rightEncoder.getDistance());
				//				straightDriveAngle(25, 0.5, -15, true);
				//				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case (4):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(10, .5, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .4) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case (5):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(10, .3, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .8) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(6):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				if ((Timer.getFPGATimestamp() - matchTimer) > .5) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case (7):
					straightDriveAngle(28, 0.25, 0, true);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
				if (matchTimer == 0) {
					matchTimer = Timer.getFPGATimestamp();
				}
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(8):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				straightDriveAngle(0, 0, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(9):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(-25, -0.65, -35, false);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.25) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(10):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}

				if (rotateDrive(0) || ((Timer.getFPGATimestamp() - matchTimer) > 2)) {
					autoCase++;
					firstFlag = true;
					matchTimer = 0;
				}
				break;
				case(11):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(25, 0.35, 0, true);
				grabberArm.setAngleArm(AngleState.kLowered, 0.3);
				grabberArm.cubeIntake(IntakeMode.kAutomatic, cubeIntakeSpeed);
				if ((Timer.getFPGATimestamp() - matchTimer) > 2 || !grabberArm.outerBreakBeam.get()) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(12):
					if(grabberArm.cubeIntake(IntakeMode.kAutomatic, cubeIntakeSpeed)){
						autoCase++;
						matchTimer = 0;
						firstFlag = true;
					}
				break;
				case(13):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(-25, -0.5, 0, false);
				grabberArm.setAngleArm(AngleState.kLowered, 0.3);
				grabberArm.cubeIntake(IntakeMode.kAutomatic, cubeIntakeSpeed);
				if ((Timer.getFPGATimestamp() - matchTimer) > 0.7) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(14):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kInitial, 0.7);
				if (rotateDrive(-35) || (Timer.getFPGATimestamp() - matchTimer) > 2) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(15):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kInitial, 0.4);
				straightDriveAngle(12, 0.8, -35, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .6) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(16):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kInitial, 0.4);
				straightDriveAngle(12, 0.25, -25, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;

				case(17):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				if ((Timer.getFPGATimestamp() - matchTimer) > .5) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case (18):
					straightDriveAngle(28, 0.25, 0, true);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
				if (matchTimer == 0) {
					matchTimer = Timer.getFPGATimestamp();
				}
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case (19):
					straightDriveAngle(-28, -0.4, 0, false);
				if (matchTimer == 0) {
					matchTimer = Timer.getFPGATimestamp();
				}
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase = 10000;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				default:
					grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeIntakeSpeed);
					straightDriveAngle(0, 0, 0, true);
					break;
				}
				break;
			case R:
				switch(autoCase){
				//  Drives straight 
				case(0):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .8, 30, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .7) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(1):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .5, 30, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .4) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(2):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .25, 10, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(3):
					if (matchTimer == 0){
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeLaunchSpeed);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(4):
					if (matchTimer == 0){
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(5):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				straightDriveAngle(0, 0, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.5) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(6):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(-25, -0.65, 30, false);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.2) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(7):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}

				if (rotateDrive(0) || ((Timer.getFPGATimestamp() - matchTimer) > 2)) {
					autoCase++;
					firstFlag = true;
					matchTimer = 0;
				}
				break;
				case(8):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(25, 0.35, 0, true);
				grabberArm.setAngleArm(AngleState.kLowered, 0.3);
				grabberArm.cubeIntake(IntakeMode.kAutomatic, cubeIntakeSpeed);
				if ((Timer.getFPGATimestamp() - matchTimer) > 2 || !grabberArm.outerBreakBeam.get()) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(9):
					if(grabberArm.cubeIntake(IntakeMode.kAutomatic, cubeIntakeSpeed)){
						autoCase++;
						matchTimer = 0;
						firstFlag = true;
					}
				break;
				case(10):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(-25, -0.5, 0, false);
				grabberArm.setAngleArm(AngleState.kLowered, 0.3);
				grabberArm.cubeIntake(IntakeMode.kAutomatic, cubeIntakeSpeed);
				if ((Timer.getFPGATimestamp() - matchTimer) > 0.7) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(11):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kInitial, 0.7);
				if (rotateDrive(35) || (Timer.getFPGATimestamp() - matchTimer) > 2) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(12):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kInitial, 0.4);
				straightDriveAngle(12, 0.8, 35, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > .6) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(13):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kInitial, 0.4);
				straightDriveAngle(12, 0.25, 30, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;

				case(14):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				if ((Timer.getFPGATimestamp() - matchTimer) > .5) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case (15):
					straightDriveAngle(28, 0.25, 0, true);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
				if (matchTimer == 0) {
					matchTimer = Timer.getFPGATimestamp();
				}
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(16):
					straightDriveAngle(-28, -0.4, 20, false);
				if (matchTimer == 0) {
					matchTimer = Timer.getFPGATimestamp();
				}
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.5) {
					autoCase = 10101010;
					matchTimer = 0;
					firstFlag = true;
				}
					break;
				default:
					grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeIntakeSpeed);
					straightDriveAngle(0, 0, 0, true);
					break;
				}
			}
			break;
		case kRightPosition:
			switch (fmsAutoSelected) {
			case L:
				switch(autoCase){
				case(0):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .8, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.3) {
					straightDriveAngle(0, 0, 0, true);
					autoCase++;
					firstFlag = true;
				}
				break;
				default:
					straightDriveAngle(0, 0, 0, true);
					break; 
				}
				// Put default auto code here
				break;
			case R:
				// Right auto code here
				System.out.println("Right case");
				System.out.println("R-Pos");

				// Left auto Code here
				switch(autoCase) {
				case(0):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .8, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.4) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(1):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				if (rotateDrive(-90) || ((Timer.getFPGATimestamp() - matchTimer) > 2)) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}

				break;
				case(2):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .25, -90, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(3):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				if ((Timer.getFPGATimestamp() - matchTimer) > .5) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(4):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(5):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(-10, -0.4, -90, false);
				if ((Timer.getFPGATimestamp() - matchTimer) > .3) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(6):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}

				if (rotateDrive(0) || (Timer.getFPGATimestamp() - matchTimer) > 1.5) {
					autoCase = 10000;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				default:
					grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeLaunchSpeed);
					grabberArm.setAngleArm(AngleState.kInitial, 0.3);
					straightDriveAngle(0, 0, 0, true);
					break;
				}
				break;
			}


			/*switch (autoCase) {
				// Drives straight
				case (0):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .8, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.3) {
					straightDriveAngle(0, 0, 0, true);
					matchTimer = 0;
					autoCase = 10000;
					firstFlag = true;

				}
				straightDriveAngle( 168  130, 0.6, 0, true);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case (1):
					grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				if (rotateDrive(-90)) {
					autoCase++;
					firstFlag = true;
				}
				break;
				case (2):
					straightDriveAngle(10, 0.4, -90, true);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case (3):
					straightDriveAngle(25, 0.25, -90, true);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				if(matchTimer == 0){
					matchTimer = Timer.getFPGATimestamp();
				}
				if((Timer.getFPGATimestamp() - matchTimer) > 1){
					autoCase++;
					matchTimer = 0;
				}
				break;
				case (4): // Cube delivered, drive away
					straightDriveAngle(-40, -.5, -30, false);
				break;
				case (5): // Drive towards the center
					straightDriveAngle(40, .5, -120, false);
				break;
				default:
					break;
				}
				break;

			case Default:
				// Default auto code here
				System.out.println("Default case");
				straightDriveAngle(0, 0, 0, true);
				break;
			}*/
			break;

		case kLeftPosition:
			switch (fmsAutoSelected) {
			case L:
				// Left auto Code here
				switch(autoCase) {
				case(0):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .8, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.4) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(1):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				if(rotateDrive(90) || (Timer.getFPGATimestamp() - matchTimer) > 2){
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(2):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .25, 90, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(3):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				if ((Timer.getFPGATimestamp() - matchTimer) > .5) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(4):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(5):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(-10, -0.4, 90, false);
				if ((Timer.getFPGATimestamp() - matchTimer) > .3) {
					autoCase++;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				case(6):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}

				if (rotateDrive(0) || (Timer.getFPGATimestamp() - matchTimer) > 1.5) {
					autoCase = 10000;
					matchTimer = 0;
					firstFlag = true;
				}
				break;
				default:
					grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeLaunchSpeed);
					grabberArm.setAngleArm(AngleState.kInitial, 0.3);
					straightDriveAngle(0, 0, 0, true);
					break;
				}
				break;

				/*switch (autoCase) {
				// Drives straight
				case (0):
					if (matchTimer == 0) {
						matchTimer = Timer.getFPGATimestamp();
					}
				straightDriveAngle(60, .8, 0, true);
				if ((Timer.getFPGATimestamp() - matchTimer) > 1.3) {
					straightDriveAngle(0, 0, 0, true);
					matchTimer = 0;
					autoCase = 10000;
					firstFlag = true;

				}	
					straightDriveAngle(130 210 , .8, 0, true);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case (1):
					grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				if (rotateDrive(90)) {
					autoCase++;
				}
				break;
				case (2):
					straightDriveAngle(25, 0.6, 90, true);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case (3):
					straightDriveAngle(25, 0.25, 90, true);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				grabberArm.setMotorArm(MotorState.kDeposit, 0.8, 0.7);
				if(matchTimer == 0){
					matchTimer = Timer.getFPGATimestamp();
				}
				if((Timer.getFPGATimestamp() - matchTimer) > 1){
					autoCase++;
					matchTimer = 0;
				}
				break;
				default:
					straightDriveAngle(0, 0, 0, true);
					break;*/
			}
			break;
		case R:
			switch(autoCase){
			case(0):
				if (matchTimer == 0) {
					matchTimer = Timer.getFPGATimestamp();
				}
			straightDriveAngle(60, .8, 0, true);
			System.out.println("t:"+Timer.getFPGATimestamp());
			if ((Timer.getFPGATimestamp() - matchTimer) > 1.3) {
				straightDriveAngle(0, 0, 0, true);
				autoCase++;
				firstFlag = true;
			}
			// Put default auto code here
			break;
			default:
				straightDriveAngle(0, 0, 0, true);
				break;
			}
			break;

		case kStraightLineAuton:
		default:
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
			straightDriveAngle(60, .5, 0, true);
			if ((Timer.getFPGATimestamp() - matchTimer) > 3) {
				straightDriveAngle(0, 0, 0, true);

			}
			// Put default auto code here
			break;
		}

		Timer.delay(0.05); // wait for a motor update time
	}

	/**
	 * Method for driving straight in auton
	 * 
	 * @param targetDistance
	 *            The distance you want to travel (in inches)
	 * @param speed
	 *            The speed that you want the robot to travel at (range is -1 to
	 *            1)
	 */
	public void straightDrive(double targetDistance, double speed) {

		// Resets the encoders and the distance traveled the first time this
		// enters
		if (firstFlag) {
			rightEncoder.reset();
			leftEncoder.reset();

			rDistance = 0;
			lDistance = 0;

			straightController.reset();
			straightController.enable();

			System.out.println("resetting");

			// Sets the Setpoint so the robot travels straight
			straightController.setSetpoint(0);

			firstFlag = false;
		}

		// Enables the turn controller if it is not already
		if (!straightController.isEnabled()) {
			rotateToAngleRate = 0;
			straightController.enable();
		}

		// Gets the total distance from the encoders
		// This encoder must be inverted so that we get proper values
		lDistance = leftEncoder.getDistance();
		rDistance = rightEncoder.getDistance();

		// Prints distance from encoders
		// System.out.println(rDistance + " " + lDistance);

		// Gets rate of rotation from PID
		rotateToAngleRate = straightController.get();

		// Stops robot if target distance was reached and moves to the next case
		if (/*targetDistance <= lDistance || */targetDistance <= rDistance) {
			driveTrain.arcadeDrive(0, 0, false);
			autoCase++;
			firstFlag = true;
			rDistance = 0;
			lDistance = 0;
		}

		// Drives straight forward if target is not reached
		else {
			driveTrain.arcadeDrive(speed, rotateToAngleRate, false);
		}
	}

	/**
	 * Method for driving straight in auton
	 * @param targetDistance The distance you want to travel (in inches)
	 * @param speed The speed that you want the robot to travel at (range is -1 to 1)
	 * @param driveAngle Angle that is used as "zero" when going straight after turning
	 * @param direction	The direction the robot will move where forwards is true
	 */
	public void straightDriveAngle(double targetDistance, double speed, double driveAngle, boolean direction){

		//	Resets the encoders and the distance traveled the first time this enters
		if(firstFlag){
			rightEncoder.reset();
			leftEncoder.reset();

			rDistance = 0;
			lDistance = 0;

			System.out.println("resetting");

			straightController.reset();
			straightController.enable();

			// Sets the Setpoint so the robot travels straight
			straightController.setSetpoint(driveAngle);

			firstFlag = false;
		}

		//	Enables the turn controller if it is not already
		if (!straightController.isEnabled()) {
			rotateToAngleRate = 0;
			straightController.enable();
		}

		//	Gets the total distance from the encoders
		//	This encoder must be inverted so that we get proper values
		lDistance = leftEncoder.getDistance();
		rDistance = rightEncoder.getDistance();

		//	Prints distance from encoders
		//		System.out.println("R:[" +rDistance+ "][" +rightEncoder.getRaw()+ "] L:[" +lDistance+ "][" +leftEncoder.getRaw()+ "]");

		//	Gets rate of rotation from PID
		rotateToAngleRate = straightController.get();

		if(direction){
			//	Stops robot if target distance was reached and moves to the next case
			if(targetDistance <= rDistance /*|| targetDistance <= rDistance*/){
				driveTrain.arcadeDrive(0, 0, false);

				turnController.reset();
				//				System.out.println("here");
				autoCase++;
				firstFlag = true;
				rDistance = 0;
				lDistance = 0;
			}
			//		Drives straight forward if target is not reached
			else{
				driveTrain.arcadeDrive(speed, rotateToAngleRate, false);
			}
		}
		else if (!direction){
			if(targetDistance >= rDistance || targetDistance >= lDistance){
				driveTrain.arcadeDrive(0, 0, false);

				turnController.reset();
				//				System.out.println("here");
				autoCase++;
				firstFlag = true;
				rDistance = 0;
				lDistance = 0;	
			}
			//			Drives straight forward if target is not reached
			else{
				driveTrain.arcadeDrive(speed, rotateToAngleRate, false);
			}
		}	

	}


	/**
	 * Method for rotating into the target angle in auton
	 * 
	 * @param targetAngle
	 *            The angle we want to turn to (in degrees)
	 */
	public void rotatePIDDrive(double targetAngle) {
		// If this is the first time entering this method, sets target angle
		if (firstFlag) {
			turnController.reset();
			rotateCount = 0;
			turnController.setSetpoint(targetAngle);
			/*		We were going to use this to make the turn in center auton faster
			turnController.setP(kPRotate * 2);
			 */			turnController.enable();

			 firstFlag = false;
		}

		// If the turn controller is not enabled, enable turn controller
		if (!turnController.isEnabled()) {
			// rotateRate = 0; // This value will be updated in the pidWrite()
			// method.
			turnController.enable();
		}

		// Sets the speed the the robot rotates at from the PID
		rotateToAngleRate = turnController.get();

		// Prints setpoint and rotation rate
		// System.out.println(turnController.getSetpoint());
		// System.out.println(rotateToAngleRate);
		// System.out.println(ahrs.getAngle());

		/*
		 * if(-rotateThreshold < rotateToAngleRate && rotateToAngleRate <
		 * rotateThreshold) { rotateCount++; }
		 * 
		 * // If the PID has slowed down to a certain point, exit the case
		 * if((rotateToAngleRate < rotateThreshold && rotateToAngleRate >
		 * -rotateThreshold) && rotateCount > 4){ // If we have, stop and return
		 * true autoCase++; firstFlag = true; 
		 * }
		 */

		// Makes the robot turn to angle
		driveTrain.arcadeDrive(0, rotateToAngleRate, false);
	}

	/**
	 * Method for rotating into the target angle in auton
	 * 
	 * @param targetAngle
	 *            The angle we want to turn to (in degrees)
	 */
	public boolean rotateDrive(double targetAngle) {
		boolean negativeAngle = false;
		double turnSpeed = kRotateSpeed;
		boolean doneTurning = false;

		// Flag if we're turning in the negative direction (negative Angle)
		if (targetAngle < 0)
		{
			negativeAngle = true;
			turnSpeed = -turnSpeed;
		}

		// Early outs for when we've reached target.
		if (negativeAngle){
			// If we've achieved the turn in the negative direction, exit
			if (ahrs.getAngle() <= targetAngle){
				doneTurning = true;
			}

		} else {
			// If we've achieved the turn in the positive direction, exit
			if (ahrs.getAngle() >= targetAngle){
				doneTurning = true;
			}

		}

		// System.out.println(turnSpeed);
		// System.out.println(ahrs.getAngle());

		// Makes the robot turn to angle
		if (!doneTurning) driveTrain.arcadeDrive(0, turnSpeed, false);

		return doneTurning;
	}



	public void teleopInit() {

		this.manualGrabberControl = true;

		driveTrain.setSafetyEnabled(true);

		// Resets the encoders
		//		leftEncoder.reset();
		rightEncoder.reset();

		rDistance = 0;
		lDistance = 0;
	}

	public void speedSetter(double triggerInput){
		desiredSpeed = triggerInput;
		if(desiredSpeed > (currentSpeed + kMaxDeltaSpeed)){
			currentSpeed += kMaxDeltaSpeed;
		}
		else if(desiredSpeed < (currentSpeed - kMaxDeltaSpeed)){
			currentSpeed -= kMaxDeltaSpeed;
		}
		else{
			currentSpeed = desiredSpeed;
		}
	}

	public void rotateSetter(double rotateInput) {
		desiredRotate = rotateInput;
		if(desiredRotate > (currentRotate + kMaxDeltaSpeed)){
			currentRotate += kMaxDeltaSpeed;
		}
		else if(desiredRotate < (currentRotate - kMaxDeltaSpeed)){
			currentRotate -= kMaxDeltaSpeed;
		}
		else{
			currentRotate = desiredRotate;
		}
	}
	/**
	 * The code that runs periodically while the robot is in teleop mode
	 */
	public void teleopPeriodic() {

		fieldMatchTime = Timer.getMatchTime();

		speedSetter(pilotController.getTriggerAxis(Hand.kRight) - pilotController.getTriggerAxis(Hand.kLeft));
		rotateSetter(pilotController.getX(Hand.kLeft));
		/// *
		// Drives robot based on video game style controls, rTrigger is forward,
		/// lTrigger is reverse, left stick is turning
		driveTrain.arcadeDrive(currentSpeed, currentRotate, true);
		// Timer.delay(0.05);
		//		System.out.println(grabberArm.armEncoder.getRaw());
		// Sets rdistance to the right encoder value
		lDistance = leftEncoder.getDistance();
		rDistance = rightEncoder.getDistance();
		//		System.out.println(rDistance);
		//		System.out.println(leftEncoder.getDistance());

		//		System.out.println(rightEncoder.getRate());
		// Prints the encoder data.
		//		 System.out.println("R:[" +rDistance+ "][" +leftEncoder.getRaw()+ "] L:[" +lDistance+ "][" +rightEncoder.getRaw()+ "]");
		//		System.out.println("ArmEncoder:[" + grabberArm.armEncoder.getRaw() + "]");

		//	Prints the status of the break beams
		//		System.out.println("Outer Break Beam:  " + grabberArm.outerBreakBeam.get());
		//		System.out.println("Inner Break Beam:  " + grabberArm.innerBreakBeam.get());

		// TODO More commented out crate arm closed and open
		// If armFlag is false and the A button on the copilot controller is
		// pressed, close the crate arm

		// Raises the arm if the right bumper is pressed
		if (copilotController.getBumper(Hand.kRight)) {
			grabberArm.setAngleArm(AngleState.kInitial, 0.7);
		}

		// Lowers arm if the left bumper is pressed
		if (copilotController.getBumper(Hand.kLeft)) {
			grabberArm.setAngleArm(AngleState.kLowered, 0.4);

		}


		if (!manualGrabberControl){
			// Launches cube when left trigger is pressed
			if (Math.abs(copilotController.getTriggerAxis(Hand.kLeft)) > 0.1) {
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, copilotController.getTriggerAxis(Hand.kLeft));
			}
			else {			
				grabberArm.cubeIntake(IntakeMode.kAutomatic, cubeIntakeSpeed);
			}
		}
		else {
			// If A button on copilot control is pressed, close the crate arm
			if (copilotController.getAButtonReleased()) {
				grabberArm.setGrabberArm(ArmState.kClosed);
			}

			// If B button on the copilot controller is pressed, open the crate arm
			if (copilotController.getBButtonReleased()) {
				grabberArm.setGrabberArm(ArmState.kOpen);
			}

			// If there is not a cube in the grabber and right trigger is pressed turn
			// motors on to intake cube
			if (Math.abs(copilotController.getTriggerAxis(Hand.kRight)) > 0.1) {
				grabberArm.setMotorArm(MotorState.kIntake, copilotController.getTriggerAxis(Hand.kRight), cubeLaunchSpeed);
			}
			// Launches cube when left trigger is pressed
			else if (Math.abs(copilotController.getTriggerAxis(Hand.kLeft)) > 0.1) {
				grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, copilotController.getTriggerAxis(Hand.kLeft));
			}
		}


		// Controls for the climber based on copilot pressing the left trigger
		// and right bumper
		if ((copilotController.getXButton())) {
			// TODO: update winch speeds in RobotClimber
			climber.winchControl(-.3, false);
		}

		if (copilotController.getYButton()) {
			climber.winchControl(-.7, false);
		}

		// climber.winchControl(copilotController.getTriggerAxis(Hand.kLeft),
		// false);//copilotController.getBumper(Hand.kRight));
		// System.out.println(copilotController.getTriggerAxis(Hand.kLeft));
		// If the Y stick is pressed up, extend the climber. If it is pulled
		// back, retract the climber
		// The comparison is inverted due to the Y-stick naturally being
		// inverted

		//	TODO: make a variable for getMatchTime
		if(fieldMatchTime < 30){
			System.out.println("Climer Solenoid Enabled");
			if (copilotController.getY(Hand.kLeft) < -0.8) {
				climber.setClimbSolenoid(ClimbState.kExtend);
				// System.out.println("here");
			} else if (copilotController.getY(Hand.kLeft) > 0.8) {
				climber.setClimbSolenoid(ClimbState.kRetract);
			}
		}
		if (copilotController.getStartButtonReleased()) {
			manualGrabberControl = !manualGrabberControl;
			SmartDashboard.putBoolean("Pilot Control", manualGrabberControl);
		}

	}

	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX MXP yaw angle input and PID Coefficients. */
	// This method is required but unused
	public void pidWrite(double output) {
		// rotateToAngleRate = output;
	}

	public void testInit() {
		grabberArm.setGrabberArm(ArmState.kOpen);
		// grabberArm.setMotorArm(MotorState.kIntake, cubeIntakeSpeed,
		// cubeLaunchSpeed);
		matchTimer = 0;
		testTimer = new Timer();
		testTimer.reset();
		Timer.delay(0.05);
		testTimer.start();

		// Sets initial position to the raised position
		grabberArm.armEncInit = grabberArm.armEncoder.getRaw();

		// Sets raised position to an offset of the init position
		grabberArm.armEncRaised = grabberArm.armEncInit + 80;

		testCase = 0;
	}
	// TODO Bookmark for test periodic
	public void testPeriodic() {
		//	USE THIS FOR RR AND LL AUTON
		switch(autoCase) {
		case(0):
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
		straightDriveAngle(60, .8, 0, true);
		if ((Timer.getFPGATimestamp() - matchTimer) > 1.2) {
			autoCase++;
			matchTimer = 0;
		}
		break;
		case(1):
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
		if(rotateDrive(-90) || (Timer.getFPGATimestamp() - matchTimer) > 1){
			autoCase++;
			matchTimer = 0;
		}
		break;
		case(2):
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
		straightDriveAngle(60, .25, -90, true);
		if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
			autoCase++;
			matchTimer = 0;
		}
		break;
		case(3):
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
		grabberArm.setAngleArm(AngleState.kRaised, 0.3);
		if ((Timer.getFPGATimestamp() - matchTimer) > .5) {
			autoCase++;
			matchTimer = 0;
		}
		break;
		case(4):
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
		grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
		grabberArm.setAngleArm(AngleState.kRaised, 0.3);
		if ((Timer.getFPGATimestamp() - matchTimer) > .75) {
			autoCase = 42424242;
			matchTimer = 0;
		}
		break;
		default:
			grabberArm.setMotorArm(MotorState.kOff, cubeIntakeSpeed, cubeLaunchSpeed);
			grabberArm.setAngleArm(AngleState.kInitial, 0.3);
			straightDriveAngle(0, 0, -90, true);
			break;
		}
		/*		switch(testCase){
		case(0):
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
		grabberArm.setAngleArm(AngleState.kRaised, 0.3);
		if ((Timer.getFPGATimestamp() - matchTimer) > 1) {
			testCase++;
			matchTimer = 0;
			firstFlag = true;
		}
			break;
		case(1):
			if (matchTimer == 0) {
				matchTimer = Timer.getFPGATimestamp();
			}
		grabberArm.setAngleArm(AngleState.kRaised, 0.3);
		grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, .4);
		if ((Timer.getFPGATimestamp() - matchTimer) > .5) {
			testCase++;
			matchTimer = 0;
			firstFlag = true;
		}
			break;
		default:
			grabberArm.setMotorArm(MotorState.kOff, 0, 0);
			grabberArm.setAngleArm(AngleState.kLowered, 0.3);
			break;
		}*/
		//	Prints the status of the break beams
		//		System.out.println("Outer Break Beam:  " + grabberArm.outerBreakBeam.get());
		//		System.out.println("Inner Break Beam:  " + grabberArm.innerBreakBeam.get());

		/*		if(testTimer.get() < 1 ){
			//	Grabber arm initial
			grabberArm.setAngleArm(AngleState.kInitial, 0.3);
			System.out.println("Grabber set to initial position, Encoder value: /t" + grabberArm.armEncoder.getRaw());
		}
		else if(testTimer.get() < 2){
			//	Grabber arm raised
			grabberArm.setAngleArm(AngleState.kRaised, 0.3);
			System.out.println("Grabber set to raised position, Encoder value: /t" + grabberArm.armEncoder.getRaw());
		}
		else if(testTimer.get() < 3){
			//	Grabber arm lower
			grabberArm.setAngleArm(AngleState.kLowered, 0.3);
			System.out.println("Grabber set to lowered position, Encoder value: /t" + grabberArm.armEncoder.getRaw());
		}*/
		/*		else if(testTimer.get() < 4){
			//	Grabber arm intake
			grabberArm.setMotorArm(MotorState.kIntake, cubeIntakeSpeed, cubeLaunchSpeed);
			System.out.println("Grabber set to intake");
		}
		else if(testTimer.get() < 5){
			//	Grabber arm deposits
			grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
			System.out.println("Grabber set to launch");
		}*/
		/*if(testTimer.get() < 1){
			//	Grabber arm opens
			grabberArm.setGrabberArm(ArmState.kOpen);
			System.out.println("Grabber set to open");
		}
		else if(testTimer.get() < 2){
			//	Grabber arm closes
			grabberArm.setGrabberArm(ArmState.kClosed);
			System.out.println("Grabber arm set to closed");
		}*/
		/*		else if(testTimer.get() < 8){
			//	Drive forward
			straightDriveAngle(20, 0.3, 0, true);
			System.out.println("Driving forwards");
			System.out.println("Left Encoder:[" + leftEncoder.getDistance() + "]   Right Encoder: [" + rightEncoder.getDistance() + "]");			
		}
		else if(testTimer.get() < 9){
			//	Drive backwards
			straightDriveAngle(-20, -0.3, 0, false);
			System.out.println("Left Encoder:[" + leftEncoder.getDistance() + "]   Right Encoder: [" + rightEncoder.getDistance() + "]");			
			System.out.println("Driving backwards");
		}
		else if(testTimer.get() < 10){
			//	Turn right
			rotateDrive(90);
			System.out.println("Left Encoder:[" + leftEncoder.getDistance() + "]   Right Encoder: [" + rightEncoder.getDistance() + "]");
			System.out.println("Turning right");
		}
		else if(testTimer.get() < 11){
			//	Turn left
			rotateDrive(0);
			System.out.println("Left Encoder:[" + leftEncoder.getDistance() + "]   Right Encoder: [" + rightEncoder.getDistance() + "]");
			System.out.println("Turning left");
		}*/
		/*		else if(testTimer.get() < 12){
			//	Tests pixy
			pixyCamera.getObjectPosition();
		}*/
		/*else if(testTimer.get() < 3) {
			//	Extends the climbing solenoid
			System.out.println("Extending climber");
			climber.setClimbSolenoid(ClimbState.kExtend);
		}
		else if(testTimer.get() < 4) {
			//	Retracts the climbing solenoid
			System.out.println("Retracting climber");
			climber.setClimbSolenoid(ClimbState.kRetract);
		}
		else if(testTimer.get() < 5) {
			//	turns off the climbing solenoid (not sure if needed, added just in case)
			System.out.println("Turning climber off");
			climber.setClimbSolenoid(ClimbState.kOff);
		}*/


		//		System.out.println("Inner	" + grabberArm.innerBreakBeam.get());
		//		System.out.println("Outer	" + grabberArm.outerBreakBeam.get());
		//		System.out.println("Left Encoder:[" + leftEncoder.getDistance() + "]   Right Encoder: [" + rightEncoder.getDistance() + "]");			

	}
}
