/*
 * Team 4454 FRC 2017 FIRST STEAMWORKS
 * 
 * http://slarobotics.org
 */


package org.usfirst.frc.team4454.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;

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

	AHRS ahrs;

	PowerDistributionPanel pdp;

	double scale;
	boolean defaultspeed;

	double shooterRPM = 0.0;

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


	// VISION DATA STRUCTURES
	UsbCamera shooterCamera, intakeCamera;
	VisionThread visionThread;
	CvSource outputStream;
	
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
	double I = 0.00001;
	double D = 0.0;


	DigitalInput GearSensor = new DigitalInput(0);

	boolean preferredRPM;	

	int n = 0;
	
	public enum AutonMode { DRIVE_STRAIGHT_1, TURN_1, FINISHED };
	AutonMode currentAutonMode = AutonMode.FINISHED;


	public Robot() {
		// The code below sets up the Joysticks and talons for the drivetrain.
		//(front or the back of the robot)(Right or Left of the robot)

		// left CAN IDs:
		// right CAN IDs: 4, 5, 6
		// shooter CAN ID: 7
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
		//Double check these numbers.
		climber = new CANTalon(10);
		intake = new CANTalon(11);

		shooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

		shooter.reverseSensor(true);
		shooter.reverseOutput(true);

		shooter.configNominalOutputVoltage(0.0f, -0.0f);
		shooter.configPeakOutputVoltage(12.0f, -12.0f);

		shooter.setProfile(0);
		shooter.setF(F);
		shooter.setP(P);
		shooter.setI(I);
		shooter.setD(D);
		shooter.changeControlMode(TalonControlMode.Speed);

		pdp = new PowerDistributionPanel(); 

		// camera paths:
		//    top USB: /dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0
		// bottom USB: /dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0

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

		shooterCamera.setExposureAuto();
		intakeCamera.setExposureAuto();

//		shooterCamera.setExposureManual(10);
//		intakeCamera.setExposureManual(10);

		outputStream = CameraServer.getInstance().putVideo("hsvThreshold", 320, 240);

		visionThread = new VisionThread(shooterCamera, new OurVisionPipeline(),
				pipeline->{
					outputStream.putFrame(pipeline.hsvThresholdOutput());

					if (!pipeline.filterContoursOutput().isEmpty()) {
					};

					if (exposureChanged) {
//						shooterCamera.setExposureManual(exposureValue);
						exposureChanged = false;
					}

					pipeline.hsvThresholdHue[0] = hueMin;
					pipeline.hsvThresholdHue[1] = hueMax;

					pipeline.hsvThresholdSaturation[0] = satMin;
					pipeline.hsvThresholdSaturation[1] = satMax;

					pipeline.hsvThresholdValue[0] = valMin;
					pipeline.hsvThresholdValue[1] = valMax;

				});

		visionThread.start();

		
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

	}
	
	// TELEOP MODE

	@Override
	public void teleopInit() {
		ahrs.zeroYaw();
	}
	
	@Override
	public void teleopPeriodic () {
		// these need to be negated because forward on the stick is negative
		double leftAxis = -leftStick.getY();
		double rightAxis = -rightStick.getY();

		double scale = getDrivePowerScale();

		adaptiveDrive(leftAxis * scale, rightAxis * scale);
		//setClimberMotors(rightStick.getX());
		
		if (operatorStick.getRawButton(4)) {
			setIntakeMotors(1.0);
		} else if (operatorStick.getRawButton(2)) {
			setIntakeMotors(-1.0);
		} else {
			setIntakeMotors(0);
		}
		
		if (operatorStick.getRawButton(1)) {
			shooter.set(shooterRPM);
		} else {
			shooter.set(0);
		}

		if (n%10 == 0) {
			if (operatorStick.getRawButton(6) && (shooterRPM < 6000.0))
				shooterRPM += 100.0;

			if (operatorStick.getRawButton(5) && (shooterRPM >= 0.0))
				shooterRPM -= 100.0;
		}

		report();
		n++;
		double newF = SmartDashboard.getNumber("F", F);
		if(!compareDoubles(newF, F)) {
			F = newF;
			shooter.setF(F);
			System.out.format("Updating F to %f\n", F);
		}
		double newP = SmartDashboard.getNumber("P", P);
		if(newP != P) {
			P = newP;
			shooter.setP(P);
			System.out.format("Updating P to %f\n", P);
		}
		double newI = SmartDashboard.getNumber("I", I);
		if(!compareDoubles(newI, I)) {
			I = newI;
			shooter.setI(I);
			System.out.format("Updating I to %f\n", I);
		}
		double newD = SmartDashboard.getNumber("D", D);
		if(!compareDoubles(newD, D)) {
			D = newD;
			System.out.format("Updating D to %f\n", D);
			shooter.setD(D);
		}
		/*
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
		 */
		
		if(leftStick.getRawButton(9)) {
			
		}
	}
	

	// AUTONOMOUS MODE
	
	@Override
	public void autonomousInit() {
		resetDistanceAndYaw();
		currentAutonMode = AutonMode.DRIVE_STRAIGHT_1;
	}

	@Override
	public void autonomousPeriodic() {

		switch (currentAutonMode) {

			case DRIVE_STRAIGHT_1 :
				// Adaptive Drive corrects for yaw drift
				driveStraight(0.2);
				if (encRight.getDistance() > 3.0) {
					resetDistanceAndYaw ();
					currentAutonMode = AutonMode.TURN_1;
				}
				break;
	
			case TURN_1 :
				adaptiveDrive(0.1, -0.1);
				if (ahrs.getAngle() > 90.0) {
					resetDistanceAndYaw ();
					currentAutonMode = AutonMode.FINISHED;
				}
				break;
	
			case FINISHED :
				setDriveMotors (0.0, 0.0);
				break;
		}
		
		// Report current mode on dashboard
		SmartDashboard.putString ("currentAutonMode", currentAutonMode.toString());
		
		report();
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
	
	public void adaptiveDrive(double l, double r){

		SmartDashboard.putNumber("LStick", l);
		SmartDashboard.putNumber("RStick", r);

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

		SmartDashboard.putNumber("PowerVal", c);
		SmartDashboard.putNumber("DirectionVal", d);
		SmartDashboard.putNumber("LOut", l_out);
		SmartDashboard.putNumber("ROut", r_out);
		SmartDashboard.putNumber("Corr", corr);

		setDriveMotors(l_out, r_out);
	}
	
	// driveStraight implements a simple proportional controller designed to regulate the yaw angle to zero
	public void driveStraight (double c) {
		double d = -0.01 * ahrs.getAngle();
		setDriveMotors (c+d, c-d);
	}
	
	public double getDrivePowerScale() {
		scale = 0.65;

		defaultspeed = false;

		if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
			scale = 0.85;
		}
		if (leftStick.getTrigger() && rightStick.getTrigger()) {
			scale = 1;
			shooter.set(0);
			climber.set(0);
			intake.set(0);
		} 
		if(scale == 0.65){
			defaultspeed = true;
		}

		return scale;
	}

 	public void reportEncoders() {
		SmartDashboard.putNumber("Left Count", countL);
		SmartDashboard.putNumber("Left Raw Distance", rawDistanceL);
		SmartDashboard.putNumber("Left Distance", distanceL);
		SmartDashboard.putNumber("Left Rate", rateL);
		SmartDashboard.putBoolean("Left Direction", directionL);
		SmartDashboard.putBoolean("Left Stopped", stoppedL);

		SmartDashboard.putNumber("Right Count", countR);
		SmartDashboard.putNumber("Right Raw Distance", rawDistanceR);
		SmartDashboard.putNumber("Right Distance", distanceR);
		SmartDashboard.putNumber("Right Rate", rateR);
		SmartDashboard.putBoolean("Right Direction", directionR);
		SmartDashboard.putBoolean("Right Stopped", stoppedR);
	}

	public void reportShooters() {
		SmartDashboard.putNumber(" J", shooterRPM);
		SmartDashboard.putNumber(" V", shooter.getOutputVoltage());
		SmartDashboard.putNumber("SP", shooter.getSetpoint());
		SmartDashboard.putNumber(" S", shooter.getSpeed());
		SmartDashboard.putNumber(" E", shooter.getError());

		if (shooterRPM == 3250) {
			preferredRPM = true;
		} else {
			preferredRPM = false;
		}

		SmartDashboard.putBoolean("Preferred RPM", preferredRPM);
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

	public void reportSpeed() {
		SmartDashboard.putNumber("Speed", scale);
		SmartDashboard.putBoolean("Default Speed", defaultspeed);
	}

	public void reportPower() {
		double current = pdp.getTotalCurrent();
		double power = pdp.getTotalPower();
		
		SmartDashboard.putNumber("Current", current);
		SmartDashboard.putNumber("Power", power);
	}

	public void report() {
		reportShooters();
		reportAhrs();
		updateEncoders();
		reportEncoders();
		reportSpeed();
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

	public void setClimberMotors(double climberVal) {
		if (climberVal > 0.05 || climberVal < -0.05) {
			climber.set(climberVal * 7000);
		}
		else {
			climber.set(0);
		}
	}

	public void setIntakeMotors(double intakeVal) {
		if (intakeVal > 0.05 || intakeVal < -0.05) {
			intake.set(intakeVal * 7000);
		}
		else {
			intake.set(0);
		}
	}
	
	public static boolean compareDoubles(double expected, double actual)
	{
		return Math.abs(expected-actual) < 5 * Math.ulp(expected);
	}

}
