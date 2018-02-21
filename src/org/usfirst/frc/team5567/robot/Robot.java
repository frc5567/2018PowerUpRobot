package org.usfirst.frc.team5567.robot;

import org.usfirst.frc.team5567.robot.Grabber.AngleState;
import org.usfirst.frc.team5567.robot.Grabber.ArmState;
import org.usfirst.frc.team5567.robot.Grabber.MotorState;
import org.usfirst.frc.team5567.robot.RobotClimber.ClimbState;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
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
	//	global variables

	//	Sets the type of grabber where true is motor and false is Solonoid
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
	double rotateThreshold = 0.1;

	//	Rotational Constants for turning x degrees where x is kTargetAngleDegrees
	double kDRotate = 0.001;
	final double kFRotate = 0.00;

	double kPRotate = 0.0067;
	double kIRotate = 0.000007;

	//	Constants for PID Controller for moving straight
	double kDStraight = 0.105;
	final double kFStraight = 0.00;

	double kPStraight = 0.0175;
	double kIStraight = 0.000212;

	//	Declaring PID variables 
	double testSpd = .4;
	double newSpd = 0;
	static final double kToleranceDegrees = 1;    
	static final double kTargetAngleDegrees = 90;

	//	Declaring the USB Camera
	UsbCamera camera;
	UsbCamera cameraTwo;

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
	private String m_dashboardAutoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	//	Instantiates a value that should equal 90 degrees on the encoder that we will use to break a while loop
	final int kInitAngle = -130;
	/*
	 * This is our robot's constructor.
	 */
	public Robot() {
		//  Instantiating Speed Controllers and assigned ports
		frontLeftMotor = new VictorSP(0);
		backLeftMotor = new VictorSP(1);
		frontRightMotor = new VictorSP(2);
		frontRightMotor.setInverted(false);
		backRightMotor = new VictorSP(3);

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
		turnController.setOutputRange(-.3, .3);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		turnController.disable();

		//	Sets up the straight PID
		straightController = new PIDController(kPStraight, kIStraight, kDStraight, kFStraight, ahrs, this, 0.02);
		straightController.setInputRange(-180.0f,  180.0f);
		straightController.setOutputRange(-.3, .3);
		straightController.setAbsoluteTolerance(kToleranceDegrees);
		straightController.setContinuous(true);
		straightController.disable();


		/* Add the PID Controller to the Test-mode dashboard, allowing manual  */
		/* tuning of the Turn Controller's P, I and D coefficients.            */
		/* Typically, only the P value needs to be modified.                   */
		SmartDashboard.putNumber("KP", turnController.getP());
		SmartDashboard.putNumber("KI", turnController.getI());
		SmartDashboard.putNumber("KD", turnController.getD());
		SmartDashboard.putNumber("Speed", testSpd);

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
		climber = new RobotClimber(6,7,0,1);

	}

	@Override
	public void robotInit(){

		//	Sets up the camera stream
		camera = CameraServer.getInstance().startAutomaticCapture(0);
		cameraTwo = CameraServer.getInstance().startAutomaticCapture(1);
		camera.setResolution(160, 120);
		camera.setFPS(1);
		cameraTwo.setResolution(320, 240);
		cameraTwo.setFPS(30);

		//  Sets up distance for pulse so getDistance is set in inches
		leftEncoder.setDistancePerPulse(0.0092);
		rightEncoder.setDistancePerPulse(0.0092);

		//	Adds options to the sendable chooser
		m_chooser.addDefault("No Move Auton", kNoMoveAuton);
		m_chooser.addObject("Right Position", kRightPosition);
		m_chooser.addObject("Left Position", kLeftPosition);

		SmartDashboard.putData("Auto choices", m_chooser);
		climber.setClimbSolenoid(ClimbState.kOff);
		grabberArm.setGrabberArm(ArmState.kOpen);

		Timer.delay(1);
		grabberArm.armEncoder.reset();
		System.out.println("Reset the encoder on the CrateGrabber");
		Timer.delay(2);
	}

	public void autonomousInit(){

		//	Zeros our gyro
		ahrs.zeroYaw();

		//	Sets the encoder distances to zero
		lDistance = 0;
		rDistance = 0;

		//  Gets the string from the FMS that shows which side of the switch and scale
		//  This is mainly a failsafe if the string doesn't get to the robot
		//  This is the failsafe so the fatal error doesn't kill the robot
		try {
			gameData = DriverStation.getInstance().getGameSpecificMessage();
		}
		catch (RuntimeException ex ) {
			DriverStation.reportError("Error retrieveing switch and scale data from FMS:  " + ex.getMessage(), true);
			fmsAutoSelected = Default;
		}

		//	Activates safety for the drive train
		driveTrain.setSafetyEnabled(true);

		//	Zeros the autocase
		autoCase = 0;

		//	Gets FMS data, chooses Auto case based on it
		if(gameData.charAt(0) == 'L') {
			//	Sets robot to use code to go left
			fmsAutoSelected = L;
			System.out.println("Left Auto Selected");
		}
		else if(gameData.charAt(0) == 'R') {
			//	Sets robot to use code to go right
			fmsAutoSelected = R;
			System.out.println("Right Auto Selected");
		}
		else {
			//	Sets robot to use default code
			fmsAutoSelected = Default;
			System.out.println("Default Auto Selected");
		}
		m_dashboardAutoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_dashboardAutoSelected);

	}

	public void autonomousPeriodic(){

		switch (m_dashboardAutoSelected) {
		case kRightPosition:
			switch(fmsAutoSelected) {
			case L:
				//	Left auto Code here
				switch(autoCase){
				//  Drives straight
				case(0):
					StraightDriveAngle(210, 0.6, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					RotateDrive(-100);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					StraightDriveAngle(140, 0.4, -100);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(3):
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
				//  Drives straight
				case(0):
					StraightDriveAngle(168, 0.6, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					RotateDrive(-90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					StraightDriveAngle(10, 0.4, -90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(3):
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

		case kLeftPosition:
			switch(fmsAutoSelected) {
			case L:
				//	Left auto Code here
				switch(autoCase){
				//  Drives straight
				case(0):
					StraightDriveAngle(210, 0.6, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					RotateDrive(100);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					StraightDriveAngle(140, 0.4, 100);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(3):
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
				//  Drives straight
				case(0):
					StraightDriveAngle(168, 0.6, 0);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(1):
					RotateDrive(90);
				grabberArm.setAngleArm(AngleState.kInitial, 0.3);
				break;
				case(2):
					StraightDriveAngle(10, 0.4, 90);
				grabberArm.setAngleArm(AngleState.kRaised, 0.3);
				break;
				case(3):
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
			straightController.setSetpoint(0);

			firstFlag = false;
		}

		//	Enables the turn controller if it is not already
		if (!straightController.isEnabled()) {
			rotateToAngleRate = 0;
			straightController.enable();
		}

		//	Gets the total distance from the encoders
		//	This encoder must be inverted so that we get proper values
		rDistance = rightEncoder.getDistance();
		lDistance = leftEncoder.getDistance();

		//	Prints distance from encoders
		//		System.out.println(rDistance + "   " + lDistance);

		//	Gets rate of rotation from PID
		rotateToAngleRate = straightController.get();

		//	Stops robot if target distance was reached and moves to the next case
		if(targetDistance <= lDistance || targetDistance <= rDistance){
			driveTrain.arcadeDrive(0, 0, false);
			autoCase++;
			firstFlag = true;
			rDistance = 0;
			lDistance = 0;
		}

		//	Drives straight forward if target is not reached
		else{
			driveTrain.arcadeDrive(speed, rotateToAngleRate, false);
		}
	}

	/**
	 * Method for driving straight in auton
	 * @param targetDistance The distance you want to travel (in inches)
	 * @param speed The speed that you want the robot to travel at (range is -1 to 1)
	 * @param driveAngle Angle thaat is used as "zero" when going straight after turning
	 */
	public void StraightDriveAngle(double targetDistance, double speed, double driveAngle){

		//	Resets the encoders and the distance traveled the first time this enters
		if(firstFlag){
			leftEncoder.reset();
			rightEncoder.reset();

			rDistance = 0;
			lDistance = 0;

			System.out.println("resetting");

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
		rDistance = rightEncoder.getDistance();
		lDistance = leftEncoder.getDistance();

		//	Prints distance from encoders
		System.out.println("R:[" +rDistance+ "][" +rightEncoder.getRaw()+ "] L:[" +lDistance+ "][" +leftEncoder.getRaw()+ "]");

		//	Gets rate of rotation from PID
		rotateToAngleRate = straightController.get();

		//	Stops robot if target distance was reached and moves to the next case
		if(targetDistance <= lDistance || targetDistance <= rDistance){
			driveTrain.arcadeDrive(0, 0, false);
			autoCase++;
			firstFlag = true;
			rDistance = 0;
			lDistance = 0;
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
			turnController.reset();
			rotateCount = 0;
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

		if(-rotateThreshold < rotateToAngleRate && rotateToAngleRate < rotateThreshold) {
			rotateCount++;
		}

		//	If the PID has slowed down to a certain point, exit the case
		if((rotateToAngleRate < rotateThreshold && rotateToAngleRate > -rotateThreshold) && rotateCount > 4){
			//	If we have, stop and return true
			autoCase++;
			firstFlag = true;
		}

		//	Makes the robot turn to angle
		driveTrain.arcadeDrive(0, rotateToAngleRate, false);
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

		System.out.println(rightEncoder.getRate());
		//	Prints the encoder data.
		//		System.out.println("R:[" +rDistance+ "][" +rightEncoder.getRaw()+ "] L:[" +lDistance+ "][" +leftEncoder.getRaw()+ "]");

		//			TODO More commented out crate arm closed and open
		//  If armFlag is false and the A button on the copilot controller is pressed, close the crate arm
		if(copilotController.getAButtonReleased()){
			grabberArm.setGrabberArm(ArmState.kClosed);
		}

		//  If B button on the copilot controller is pressed, open the crate arm
		if(copilotController.getBButtonReleased()){
			grabberArm.setGrabberArm(ArmState.kOpen);
		}

		//	Raises the arm if the right bumper is pressed
		if(copilotController.getBumper(Hand.kRight)){
			grabberArm.setAngleArm(AngleState.kRaised, -0.5);
		}

		//	Lowers arm if the left bumper is pressed
		if(copilotController.getBumper(Hand.kLeft)){
			grabberArm.setAngleArm(AngleState.kLowered, 0.3);
		}

		//	TODO Cube grabber in Teleop 
		//	If there is not a cube in the grabber and X button is pressed turn motors on to intake cube
		if(copilotController.getXButton()){
			//			if(grabberArm.detectCube() == false){
			grabberArm.setMotorArm(MotorState.kIntake, cubeIntakeSpeed, cubeLaunchSpeed);
			//			}
			//	If cube is detected stop intake motors
			//			else if(grabberArm.detectCube()){
			//				grabberArm.stopIntake();
			//			}
		}
		//	Launches cube when Y button is pressed
		else if (copilotController.getYButton()){
			grabberArm.setMotorArm(MotorState.kDeposit, cubeIntakeSpeed, cubeLaunchSpeed);
		}

		//	Controls for the climber based on copilot pressing the left trigger and right bumper 
		if(Math.abs(copilotController.getTriggerAxis(Hand.kLeft)) > .1){
			//climber.winchControl(1, false);
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
	@Override
	/* This function is invoked periodically by the PID Controller, */
	/* based upon navX MXP yaw angle input and PID Coefficients.    */
	//	This method is required but unused
	public void pidWrite(double output) {
		//rotateToAngleRate = output;
	}


	public void testInit(){
		grabberArm.setGrabberArm(ArmState.kClosed);
	}


	public void testPeriodic(){
		if(copilotController.getAButton()){
			grabberArm.setGrabberArm(ArmState.kClosed);;
		}
		else{
			grabberArm.setGrabberArm(ArmState.kOpen);;
		}
		if(copilotController.getBButton()){
			grabberArm.dSolLeft.set(Value.kOff);
		}
		System.out.println(copilotController.getTriggerAxis(Hand.kLeft));
	}
}
