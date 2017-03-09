/*
 * Team 4454 FRC 2017 FIRST STEAMWORKS
 * 
 * http://slarobotics.org
 */


package org.usfirst.frc.team4454.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Victor;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends IterativeRobot {
	
	private static final double SHOOTER_RPM_PRE_AUTON=5000.0;
	private static final double SHOOTER_RPM_PRE_TELEOP=4000.0;
	
	Joystick leftStick;  // set to ID 1 in DriverStation
	Joystick rightStick; // set to ID 2 in DriverStation
	Joystick operatorStick;

	CANTalon frontRight;
	CANTalon frontLeft;
	CANTalon backRight;
	CANTalon backLeft;
	CANTalon middleRight;
	CANTalon middleLeft;
	
	CANTalon shooter;
	CANTalon climber;
	CANTalon intake;
	
	Servo intakeFeed;
	
	Victor intakeLED;

	AHRS ahrs;

	PowerDistributionPanel pdp;

	DigitalInput GearSensor = new DigitalInput(0);


	Encoder encLeft;
	Encoder encRight;
	
	int countR;
	double rawDistanceR;
	double distanceR;
	double rateR;
	boolean directionR;
	boolean stoppedR;

	int countL;
	double rawDistanceL;
	double distanceL;
	double rateL;
	boolean directionL;
	boolean stoppedL;
	
	double targetDistance;


	// VISION DATA STRUCTURES
	UsbCamera shooterCamera;
	UsbCamera intakeCamera;
	VisionThread shooterVisionThread;
	VisionThread intakeVisionThread;
	VisionThread testVisionThread;
	CvSource shooterOutputStream;
	CvSource intakeOutputStream;
	CvSource testOutputStream;
	
	int exposureValue = 10;
	boolean exposureChanged = false;

	// Vision Parameters
	double hueMin = 50;
	double hueMax = 122;
	double satMin = 105;
	double satMax = 255;
	double valMin = 50;
	double valMax = 255;
	
	double F = 0.027;
	double P = 0.05;
	double I = 0.0;
	double D = 1.0;

	
	// Auton structures
	public int AutonStage;
	public enum AutonMode { START, DRIVE_STRAIGHT, DELAY, SHOOT, STOP, DRIVE_TO_DISTANCE };

	AutonMode currentAutonMode = AutonMode.START;
	public int currentAutonStage = 0;
	public long startTime; // Used to measure elapsed time
	
	double autonDistance = 2.4;
	double autonShootTime = 5.0;

	double shooterRPM = SHOOTER_RPM_PRE_AUTON;

	public Robot() {
		// The code below sets up the Joysticks and talons for the drivetrain.
		//(front or the back of the robot)(Right or Left of the robot)

		// left CAN IDs:
		// right CAN IDs: 4, 5, 6
		// shooter CAN ID: 7
		// intake CAN ID: 8
		// climber CAN ID: 9
		frontLeft   = new CANTalon(1);
		middleLeft  = new CANTalon(2);
		backLeft    = new CANTalon(3);

		frontRight  = new CANTalon(4);
		middleRight = new CANTalon(5);
		backRight   = new CANTalon(6);

		frontRight.setInverted(true);
		backRight.setInverted(true);
		middleRight.setInverted(true);


		leftStick  = new Joystick(0);
		rightStick = new Joystick(1);
		operatorStick = new Joystick(2);

		shooter = new CANTalon(7);
		intake  = new CANTalon(8);
		climber = new CANTalon(9);
		
		intakeFeed = new Servo(4);

		shooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

		shooter.reverseSensor(true);
		shooter.reverseOutput(true);

		shooter.configNominalOutputVoltage(0.0f, -0.0f);
		shooter.configPeakOutputVoltage(0.0f, -12.0f);
		
		shooter.enableBrakeMode(false);

		shooter.setProfile(0);
		shooter.setF(F);
		shooter.setP(P);
		shooter.setI(I);
		shooter.setD(D);
	//	shooter.SetVelocityMeasurementWindow(512);
		shooter.changeControlMode(TalonControlMode.Speed);
		
		// only allow climber to go in one direction
		climber.configNominalOutputVoltage(-0.0f, 0.0f);
		climber.configPeakOutputVoltage(0.0f, 12.0f);

		pdp = new PowerDistributionPanel(); 

		// camera paths:
		//    top USB: /dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0
		// bottom USB: /dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0

		intakeLED = new Victor(9);

		shooterCamera = CameraServer.getInstance().startAutomaticCapture("shooter", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");
		intakeCamera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0");
	}

	@Override
	public void robotInit() {

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
			ahrs.zeroYaw();
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		encLeft = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);

		encLeft.setMaxPeriod(1);
		encLeft.setMinRate(0.1);
		encLeft.setDistancePerPulse(1.0/((4.0 * 25.4 / .001) * Math.PI / 360.0));
		encLeft.setReverseDirection(false);
		encLeft.setSamplesToAverage(7);

		encRight = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);
		
		encRight.setMaxPeriod(1);
		encRight.setMinRate(0.1);
		encRight.setDistancePerPulse(1.0/((4.0 * 25.4 / .001) * Math.PI / 360.0));
		encRight.setReverseDirection(false);
		encRight.setSamplesToAverage(7);

		
		shooterCamera.setResolution(320, 240);
		intakeCamera.setResolution(320, 240);

		shooterOutputStream = CameraServer.getInstance().putVideo("shooterOverlay", 320, 240);
		intakeOutputStream = CameraServer.getInstance().putVideo("intakeOverlay", 320, 240);
		testOutputStream = CameraServer.getInstance().putVideo("testOverlay", 320, 240);
		
		shooterVisionThread = new VisionThread(shooterCamera, new LineVisionPipeline(),
				pipeline->{
					pipeline.g = 0;
					pipeline.b = 255;
					pipeline.r = 0;
					pipeline.width = 2;
					shooterOutputStream.putFrame(pipeline.lineOutput());
				});

		intakeVisionThread = new VisionThread(intakeCamera, new LineVisionPipeline(),
				pipeline->{
					pipeline.g = 0;
					pipeline.b = 0;
					pipeline.r = 255;
					pipeline.width = 2;
					intakeOutputStream.putFrame(pipeline.lineOutput());
				});

		testVisionThread = new VisionThread(intakeCamera, new OurVisionPipeline(),
				pipeline->{
					testOutputStream.putFrame(pipeline.hsvThresholdOutput());

					if (pipeline.foundTarget) {
						targetDistance = pipeline.targetDistance;
						SmartDashboard.putNumber("Target Distance", targetDistance); 
					}
					
					if (exposureChanged) {
						intakeCamera.setExposureManual(exposureValue);
						exposureChanged = false;						
					}

					pipeline.hsvThresholdHue[0] = hueMin;
					pipeline.hsvThresholdHue[1] = hueMax;

					pipeline.hsvThresholdSaturation[0] = satMin;
					pipeline.hsvThresholdSaturation[1] = satMax;

					pipeline.hsvThresholdValue[0] = valMin;
					pipeline.hsvThresholdValue[1] = valMax;

				});
		
		testVisionThread.start();
		shooterVisionThread.start();
		intakeVisionThread.start();

		SmartDashboard.putNumber("Exposure", exposureValue);

		SmartDashboard.putNumber("hueMin", hueMin);
		SmartDashboard.putNumber("hueMax", hueMax);
		SmartDashboard.putNumber("satMin", satMin);
		SmartDashboard.putNumber("satMax", satMax);
		SmartDashboard.putNumber("valMin", valMin);
		SmartDashboard.putNumber("valMax", valMax);

		SmartDashboard.putNumber("F", F);
		SmartDashboard.putNumber("P", P);
		SmartDashboard.putNumber("I", I);
		SmartDashboard.putNumber("D", D);
		
		// Set auton parameters on smart dashboard
		reportShooters();
		SmartDashboard.putNumber("autonDistance", autonDistance);
		SmartDashboard.putNumber("autonShootTime", autonShootTime);
		
		openServo(false);
	}
	
	// TELEOP MODE

	@Override
	public void teleopInit() {
		ahrs.zeroYaw();
		intakeCamera.setExposureAuto();
		shooterCamera.setExposureAuto();
		
		shooterRPM = SHOOTER_RPM_PRE_TELEOP;
	}


	@Override
	public void teleopPeriodic () {
		// these need to be negated because forward on the stick is negative
		double leftAxis  = -leftStick.getY();
		double rightAxis = -rightStick.getY();

		double scale = getDrivePowerScale();
		adaptiveDrive(leftAxis * scale, rightAxis * scale);
		
		climber.set(operatorStick.getRawAxis(5)); // Right X of Joystick
		
		intake.set(operatorStick.getRawAxis(1)); // Left X of Joystick
		
		// shooter enable/disable
		if (operatorStick.getRawAxis(3) > 0.15) {
			shooter.set(shooterRPM);
		} else {
			shooter.set(0);
		}
		
		// intake servo enable/disable
		openServo(operatorStick.getRawAxis(2) > 0.15);
		
		// intakeLED.set(operatorStick.getRawAxis(2));
		
		// set shooter rpm
		setShooterRPM();

		report();

		double newF = SmartDashboard.getNumber("F", F);
		if(!compareDoubles(newF, F)) {
			F = newF;
			shooter.setF(F);
		}
		double newP = SmartDashboard.getNumber("P", P);
		if(newP != P) {
			P = newP;
			shooter.setP(P);
		}
		double newI = SmartDashboard.getNumber("I", I);
		if(!compareDoubles(newI, I)) {
			I = newI;
			shooter.setI(I);
		}
		double newD = SmartDashboard.getNumber("D", D);
		if(!compareDoubles(newD, D)) {
			D = newD;
			shooter.setD(D);
		}
		
		int exposureValueNew = (int) SmartDashboard.getNumber("Exposure", exposureValue);
		if(exposureValueNew != exposureValue) {
			exposureValue = exposureValueNew;
			exposureChanged = true;
		}

		hueMin = SmartDashboard.getNumber("hueMin", hueMin);
		hueMax = SmartDashboard.getNumber("hueMax", hueMax);
		satMin = SmartDashboard.getNumber("satMin", satMin);
		satMax = SmartDashboard.getNumber("satMax", satMax);
		valMin = SmartDashboard.getNumber("valMin", valMin);
		valMax = SmartDashboard.getNumber("valMax", valMax);
		
	}
	
	public void openServo(boolean open) {
		if (open) {
			intakeFeed.set(0.0);
		} else {
			intakeFeed.set(0.5);
		}
	}
	
	public double getDrivePowerScale() {
		double scale = 0.65;

		if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
			scale = 0.85;
		}
		
		if (leftStick.getTrigger() && rightStick.getTrigger()) {
			scale = 1;
		}
		
		return scale;
	}


	// AUTONOMOUS MODE
	
	@Override
	public void autonomousInit() {
		currentAutonStage = 1;
		currentAutonMode = AutonMode.START;
		
		// Read auton parameters from smart dashboard
		// Read shooter RPM
		shooterRPM = SmartDashboard.getNumber("shooterRPM", shooterRPM);
		
		// Read auton distance +ve or -ve if 0 not movement
		autonDistance = SmartDashboard.getNumber("autonDistance", autonDistance);
		
		// Set shoot time in seconds if <= 0 no shot
		autonShootTime = SmartDashboard.getNumber("autonShootTime", autonShootTime);
	}
	

	@Override
	public void autonomousPeriodic() {
		switch (currentAutonStage) {
		case 1:
			AutonShoot (shooterRPM, autonShootTime);
			break;
		case 2:
			AutonDriveStraight (0.4, autonDistance);
			break;
		case 3:
			AutonHopper(0.4);
			break;
		case 4:
			AutonGear(0.4);
			break;
		}
		
		// Advance the stage after each action
		if (currentAutonMode == AutonMode.STOP) {
			++currentAutonStage;
			currentAutonMode = AutonMode.START;
		}
		
		SmartDashboard.putNumber ("currentAutonStage", currentAutonStage);
		SmartDashboard.putString ("currentAutonMode", currentAutonMode.toString());
		report();

	}
	
	public void AutonHopper (double power) {
		
	}
	
	public void AutonGear(double power) {
		
		if (targetDistance == 0) {
			currentAutonMode = AutonMode.STOP;
			return;
		}
		
		switch (currentAutonMode) {
		case START: 
			resetDistanceAndYaw(); 
			currentAutonMode = AutonMode.DRIVE_STRAIGHT;
			break;
		case DRIVE_TO_DISTANCE:
			driveStraight(Math.signum(targetDistance) * Math.abs(power));
			if (Math.abs(encRight.getDistance()) > Math.abs(targetDistance)) { // We probably need to change this number a little bit.
				//Release Gear here Delay may be needed for servos
				driveStraight(0.0); // We will probably have to make it go backwards before stopping.
				currentAutonMode = AutonMode.STOP;
			}
			break;
		default:
			break;
		}
	}
	
	public void AutonShoot (double RPM, double shootTime) {
		
		if (shootTime <= 0) {
			currentAutonMode = AutonMode.STOP;
			return;
		}
		
		switch (currentAutonMode) {
		case START:
			resetTimer();
			currentAutonMode = AutonMode.DELAY;
			shooter.set(RPM);
			break;
		case DELAY:
			// Give the shooter a second to spin up
			if (elapsedTime() >= 1.0) {		
				shooter.configNominalOutputVoltage(0.0f, -0.0f);
				shooter.configPeakOutputVoltage(0.0f, -12.0f);
				resetTimer();
				currentAutonMode = AutonMode.SHOOT;
				openServo(true);
			}
			break;
		case SHOOT:
			if (elapsedTime() >= shootTime) {
				currentAutonMode = AutonMode.STOP;
				shooter.set(0);
				openServo(false);
			}
			break;
		default:
			break;
		}
	}

	public void AutonDriveStraight (double power, double distance) {
		
		if (distance == 0) {
			currentAutonMode = AutonMode.STOP;
			return;
		}
		
		switch (currentAutonMode) {
		case START: 
			resetDistanceAndYaw(); 
			currentAutonMode = AutonMode.DRIVE_STRAIGHT;
			break;
		case DRIVE_STRAIGHT :
			driveStraight(Math.signum(distance) * Math.abs(power));
			if (Math.abs(encRight.getDistance()) > Math.abs(distance)) {
				driveStraight(0.0);
				currentAutonMode = AutonMode.STOP;
			}
			break;
		default:
			break;
		}
	}
	
	public void resetTimer () {
		startTime = System.nanoTime();
	}
	
	// Return elapsed time since last reset in seconds
	public double elapsedTime () {
		return ( (System.nanoTime() - startTime) * 1.0e-9 );
	}

	
	void resetDistanceAndYaw () {
		ahrs.zeroYaw();
		encLeft.reset();
		encRight.reset();
	}

	// UTILITY METHODS

	public void setDriveMotors(double l, double r) {
		frontRight.set(r);
		backRight.set(r);
		middleRight.set(r);

		frontLeft.set(l);
		backLeft.set(l);
		middleLeft.set(l);
	}
	
	int rpmButton = -1;
	int shooterRPMIter = 0;
	public void setShooterRPM() {
		int button;
		// set shooter rpm
		button = operatorStick.getPOV();
		if ( ( (rpmButton != 0) && (button == 0) ) || ((rpmButton == 0) && (shooterRPMIter%10 == 0)) ) {
			if(shooterRPM < 6000) shooterRPM += 100;
			shooterRPMIter = 0;
		}
		
		if ( ( (rpmButton != 180) && (button == 180) ) || ((rpmButton == 180) && (shooterRPMIter%10 == 0)) ) {
			if (shooterRPM > 0) shooterRPM -= 100;
			shooterRPMIter = 0;
		}
		rpmButton = button;
		++shooterRPMIter;
	}
	public void adaptiveDrive(double l, double r){

		// alpha is a parameter between 0 and 1
		final double alpha = 0.5;
		double c = 0.5 * (l+r);
		double d = 0.5 * (l-r);
		double scale = (1 - (alpha * c * c));
		d *= scale;

		// GYRO CORRECTION -- high if d is close to zero, low otherwise
		double gRate = ahrs.getRate();
		final double CORR_COEFF = 0.5;
		double corr = 0.0;
		if (Math.abs(d) < 0.05)
			corr = (gRate * Math.abs(c) * CORR_COEFF * (1-Math.abs(d)));
		d -= corr;

		double l_out = c + d;
		double r_out = c - d;

		setDriveMotors(l_out, r_out);
	}
	
	// driveStraight implements a simple proportional controller designed to regulate the yaw angle to zero
	public void driveStraight (double c) {
		double d = -0.01 * ahrs.getAngle();
		setDriveMotors (c+d, c-d);
	}
	
 	public void reportEncoders() {
 		//SmartDashboard.putNumber("Left Distance", distanceL);
 		SmartDashboard.putNumber("Right Distance", distanceR);
 		/*
		SmartDashboard.putNumber("Left Count", countL);
		SmartDashboard.putNumber("Left Raw Distance", rawDistanceL);
		SmartDashboard.putNumber("Left Distance", distanceL);
		SmartDashboard.putNumber("Left Rate", rateL);
		SmartDashboard.putBoolean("Left Direction", directionL);
		SmartDashboard.putBoolean("Left Stopped", stoppedL);

		SmartDashboard.putNumber("Right Count", countR);
		SmartDashboard.putNumber("Right Raw Distance", rawDistanceR);
		SmartDashboard.putNumber("Right Rate", rateR);
		SmartDashboard.putBoolean("Right Direction", directionR);
		SmartDashboard.putBoolean("Right Stopped", stoppedR);
		*/
	}

	public void reportShooters() {
		SmartDashboard.putNumber("shooterRPM", shooterRPM);
		SmartDashboard.putNumber("shooterSpeed", shooter.getSpeed());

		/*
		 * SmartDashboard.putNumber(" V", shooter.getOutputVoltage());
		 * SmartDashboard.putNumber("SP", shooter.getSetpoint());
		 * SmartDashboard.putNumber(" E", shooter.getError());
		 */
	}

	public void reportAhrs() {
		/* Display 6-axis Processed Angle Data                                      */
		SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
		SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
		SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
		SmartDashboard.putNumber(   "IMU_YawRate",          ahrs.getRate());
		SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
		SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());

		/* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
		SmartDashboard.putNumber(   "IMU_FusedHeading",     ahrs.getFusedHeading());

		SmartDashboard.putNumber(   "IMU_TotalYaw",         ahrs.getAngle());


		/* Omnimount Yaw Axis Information                                           */
		/* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
		AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
		SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
		SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );

	}

	public void reportPower() {
		double current = pdp.getTotalCurrent();
		double power = pdp.getTotalPower();
		
		SmartDashboard.putNumber("Current", current);
		SmartDashboard.putNumber("Power", power);
	}

	public void report() {
		reportShooters();
		// reportAhrs();
		updateEncoders();
		reportEncoders();
		reportPower();
	}

	public void updateEncoders() {
		countR = encRight.get();
		rawDistanceR = encRight.getRaw();
		distanceR = encRight.getDistance();
		rateR = encRight.getRate();
		directionR = encRight.getDirection();
		stoppedR = encRight.getStopped();

		countL = encLeft.get();
		rawDistanceL = encLeft.getRaw();
		distanceL = encLeft.getDistance();
		rateL = encLeft.getRate();
		directionL = encLeft.getDirection();
		stoppedL = encLeft.getStopped();
	}
	
	public static boolean compareDoubles(double expected, double actual)
	{
		return Math.abs(expected-actual) < 5 * Math.ulp(expected);
	}
	
	@Override
	public void disabledPeriodic() {
		setShooterRPM();
		report();
	}

}
