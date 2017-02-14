package org.usfirst.frc.team4454.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends SampleRobot {
	Joystick leftStick;  // set to ID 1 in DriverStation
	Joystick rightStick; // set to ID 2 in DriverStation
	Joystick operatorStick;

	Talon frontRight;
	Talon frontLeft;
	Talon backRight;
	Talon backLeft;
	Talon middleRight;
	Talon middleLeft;
	CANTalon shooter;
	CANTalon climber;

	AHRS ahrs;

	Encoder encLeft;
	Encoder encRight;

	PowerDistributionPanel pdp;

	double scale;
	boolean defaultspeed;

	double shooterRPM = 0.0;
	double climberRPM = 0.0;

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

	double current;
	double power;
	
	int contours;

	VisionThread visionThread;
	int exposureValue = 10;
	
	UsbCamera camera;
	
	DigitalInput BallSensor = new DigitalInput(0);
	
	CvSource outputStream;
	
	int n = 0;
	
	int mode = 1;
	int gearPosition = 1;
	boolean teamRed = true;
	
	boolean preferredRPM;

	public Robot() {
		// The code below sets up the Joysticks and talons for the drivetrain.
		//(front or the back of the robot)(Right or Left of the robot)

/****
		frontRight  = new Talon(0);
		frontLeft   = new Talon(4);
		backRight   = new Talon(2);
		backLeft    = new Talon(5);
		middleRight = new Talon(1);
		middleLeft  = new Talon(3);

		frontRight.setInverted(true);
		backRight.setInverted(true);
		middleRight.setInverted(true);


		leftStick  = new Joystick(0);
		rightStick = new Joystick(1);
		operatorStick = new Joystick(2);

		shooter = new CANTalon(9);
		climber = new CANTalon(10);

		shooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

		shooter.reverseSensor(false);
		shooter.reverseOutput(true);

		shooter.configNominalOutputVoltage(0.0f, -0.0f);
		shooter.configPeakOutputVoltage(12.0f, -12.0f);

		shooter.setProfile(0);
		shooter.setF(0);
		shooter.setP(.05);
		shooter.setI(.0001);
		shooter.setD(0.0);
		shooter.changeControlMode(TalonControlMode.Speed);

		pdp = new PowerDistributionPanel(); 
		
****/
		camera = CameraServer.getInstance().startAutomaticCapture();
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
		
		/***

		encLeft = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);

		encLeft.setMaxPeriod(1);
		encLeft.setMinRate(10);
		encLeft.setDistancePerPulse(5);
		encLeft.setReverseDirection(true);
		encLeft.setSamplesToAverage(7);

		encRight = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);

		encRight.setMaxPeriod(1);
		encRight.setMinRate(10);
		encRight.setDistancePerPulse(5);
		encRight.setReverseDirection(false);
		encRight.setSamplesToAverage(7);

		current = pdp.getTotalCurrent();
		power = pdp.getTotalPower();

		 ***/
		
		camera.setResolution(640, 480);
		
		camera.setExposureManual(10);
		
		outputStream = CameraServer.getInstance().putVideo("hsvThreshold", 640, 480);

		findContours();

		SmartDashboard.putNumber("Exposure", exposureValue);
	}
	
	public void findContours() {
		visionThread = new VisionThread(camera, new OurVisionPipeline(), 
				pipeline->{
					outputStream.putFrame(pipeline.hsvThresholdOutput());

					if (!pipeline.filterContoursOutput().isEmpty()) {
						contours += 1;
					};					
				});

		visionThread.start();
	}

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

	public void teleopInit() {
		ahrs.zeroYaw();
	}

	public void autonomousInit() {
		ahrs.zeroYaw();
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
		SmartDashboard.putNumber("Current", current);
		SmartDashboard.putNumber("Power", power);
	}

	public void reportGear() {
		SmartDashboard.putBoolean("Gear?", BallSensor.get());
	}
	
	public void report() {
		reportShooters();
		reportAhrs();
		updateEncoders();
		reportEncoders();
		reportSpeed();
		reportGear();
	}

	public void placeAndShoot() {

	}

	public void placeAndCross() {

	}

	public void hoppers() {

	}

	public void moveForward() {
		adaptiveDrive(0.65, 0.65);
	}

	public void autonomousPeriodic() {
		while (isAutonomous() && isEnabled()) {
			switch(mode) {
			case 1: placeAndShoot(); break;
			case 2: placeAndCross(); break;
			case 3: hoppers(); break;
			case 4: moveForward(); break;
			default: break;
			}
		}
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

	public double getScale() {
		scale = 0.65;

		defaultspeed = false;

		if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
			scale = 0.85;
		}
		if (leftStick.getTrigger() && rightStick.getTrigger()) {
			scale = 1;
			shooter.set(0);
			climber.set(0);
		} 
		if(scale == 0.65){
			defaultspeed = true;
		}

		return scale;
	}

	public void operatorControl () {
		int n = 0;
		while (isOperatorControl() && isEnabled()) {

			// these need to be negated because forward on the stick is negative
			double leftAxis = -leftStick.getY();
			double rightAxis = -rightStick.getY();

			adaptiveDrive(leftAxis * getScale(), rightAxis * getScale());

			double climberVal = rightStick.getX();
			if (climberVal > 0.05 || climberVal < -0.05) {
				climber.set(climberVal * 7000);
			}
			else {
				climber.set(0);
			}

			if (leftStick.getRawButton(2)) {
				shooter.set(shooterRPM);
			}

			if (n%100 == 0) {
				if (leftStick.getRawButton(6) && (shooterRPM < 6000.0))
					shooterRPM += 250.0;

				if (leftStick.getRawButton(7) && (shooterRPM >= 0.0))
					shooterRPM -= 500.0;
			}
			n++;

			exposureValue = (int) SmartDashboard.getNumber("Exposure", exposureValue);
			camera.setExposureManual(exposureValue);

			report();
		}
	}

	public void test() {
		while (isTest() && isEnabled()) {

			if ((n % 100) == 0) {
				exposureValue = (int) SmartDashboard.getNumber("Exposure", exposureValue);
				
				System.out.println ("exposure = " + exposureValue);
				
				if (exposureValue > 0) {
					// camera.setExposureManual(exposureValue);
				}
			}
			++n;
		}
	}

}
