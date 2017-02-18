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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends IterativeRobot {
	
	final int camWidth = 640;
	final int camHeight = 480;
	final Point p1Rect = new Point(100, 100);
	final Point p2Rect = new Point(300, 300);
	final Scalar colorRect = new Scalar(255, 0, 0);

	Joystick joyIzq, joyDer;
	Spark colgador;
	RobotDrive chasis;
	boolean finishedAuto;
	
	public void robotInit() {
		joyIzq = new Joystick(0);
		joyDer = new Joystick(1);
		colgador = new Spark(4);
		chasis = new RobotDrive(0,1,2,3);
		finishedAuto = false;
		
		new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(camWidth, camHeight);
			
			CvSink cvSink = CameraServer.getInstance().getVideo();
			CvSource outputStream = CameraServer.getInstance().putVideo("GearMonitor", camWidth, camHeight);
			
			Mat image = new Mat();
			
			while (!Thread.interrupted()) {
				cvSink.grabFrame(image);
				Imgproc.rectangle(image, p1Rect, p2Rect, colorRect);
				outputStream.putFrame(image);
			}
		}).start();
	}


	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
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
			chasis.tankDrive(joyIzq, joyDer);
			
			if(joyIzq.getRawButton(1)){
				colgador.set(0.8);
			}
			else{
				colgador.set(0);
			}
		}
	}

}

