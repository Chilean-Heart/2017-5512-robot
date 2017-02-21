package com.valentin.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;


public class Robot extends IterativeRobot {
	
	boolean finishedAuto = false;
	double Kc = 0.95;
	
	final int camWidth = 640;
	final int camHeight = 480;
	final Point p1Rect = new Point(100, 100);
	final Point p2Rect = new Point(300, 300);
	final Scalar colorRect = new Scalar(255, 0, 0);

	Joystick joyIzq, joyDer;
	Spark colgador;
	RobotDrive chasis;
	PowerDistributionPanel pdp;
	GyroAccelFilter complGyro;
	SmartDrive robot;
	
	public void robotInit() {
		// Initialize all objects
		joyIzq = new Joystick(0);
		joyDer = new Joystick(1);
		colgador = new Spark(4);
		chasis = new RobotDrive(0,1,2,3);
		pdp = new PowerDistributionPanel();
		complGyro = new GyroAccelFilter(Kc);
		robot = new SmartDrive(chasis, complGyro);
		// Invert the necessary motors. Comment out these lines if necessary
		chasis.setInvertedMotor(MotorType.kFrontLeft, true);
		chasis.setInvertedMotor(MotorType.kFrontRight, true);
		chasis.setInvertedMotor(MotorType.kRearLeft, true);
		chasis.setInvertedMotor(MotorType.kRearRight, true);
		// Start the camera thread
		new Thread(() -> {
			// Begin the automatic capture of the USB camera and set the resolution
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(camWidth, camHeight);
			// Create an input and output channel
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("GearMonitor", camWidth, camHeight);
			// Create a new image frame
			Mat image = new Mat();
			// Run this as a loop
			while (!Thread.interrupted()) {
				// Grab a new frame
				cvSink.grabFrame(image);
				// Add a rectangle
				Imgproc.rectangle(image, p1Rect, p2Rect, colorRect);
				// Output a frame
				outputStream.putFrame(image);
			}
		}).start();
	}

	@Override
	public void autonomousInit() {
		
	}

	@Override
	public void autonomousPeriodic() {
		// Run an auto routine
		if (!finishedAuto) {
			chasis.tankDrive(1,1);
			Timer.delay(2);
			chasis.tankDrive(0, 0);
			Timer.delay(0.5);
			chasis.tankDrive(-1, 1);
			Timer.delay(1.7);
			chasis.tankDrive(0, 0);
			Timer.delay(0.5);
			chasis.tankDrive(1, 1);
			Timer.delay(2);
			finishedAuto = true;
		}
	}

	@Override
	public void teleopPeriodic() {
		while(isEnabled()&&isOperatorControl()){
			// Log necessary values
			SmartDashboard.putNumber("Hanger Current", pdp.getCurrent(13));
			SmartDashboard.putNumber("Raw Gyro", complGyro.getRawAngle());
			SmartDashboard.putNumber("Complementary Gyro", complGyro.getAngle());
			SmartDashboard.putNumber("Left Speed", robot.getLeftSpeed());
			SmartDashboard.putNumber("Right Speed", robot.getRightSpeed());
			// Control the robot and the hanger
			robot.drive(joyIzq, joyDer);
			colgador.set(joyIzq.getRawButton(1) ? 1 : 0);
		}
	}

}

