package com.valentin.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;

public class SmartDrive {
	
	boolean directLock = false;
	double directHeading;
	double Kp = 0.1;
	double left, right;
	
	public RobotDrive chassis;
	GyroAccelFilter imu;
	
	public SmartDrive(RobotDrive robotBase, GyroAccelFilter inertialUnit) {
		this.chassis = robotBase;
		this.imu = inertialUnit;
	}
	
	public double getLeftSpeed() {
		return this.left;
	}
	
	public double getRightSpeed() {
		return this.right;
	}
	
	public void drive(Joystick left, Joystick right) {
		// Get speeds for tank drive
		double leftSpeed = left.getY();
		double rightSpeed = right.getY();
		// Get trigger button to enable direct drive
		boolean directDrive = right.getRawButton(1);
		// Get thumb button to enable soft drive
		boolean softDrive = right.getRawButton(2);
		if (directDrive) {
			// Lock the current heading
			if (!directLock) {
				directHeading = imu.getAngle();
				directLock = true;
			}
			// Calculate a proportional controller to correct heading
			double headingError = directHeading - imu.getAngle();
			double correction = Kp * headingError;
			// Modify side speeds
			leftSpeed += correction;
			rightSpeed -= correction;
		}
		if (softDrive) {
			// Get throttle speed and map it to an inverted 0 - 1 position
			double softSpeed = 0.5 * (1 - right.getRawAxis(3));
			// Multiply the speeds by the proportional normalized value
			leftSpeed *= softSpeed;
			rightSpeed *= softSpeed;
		}
		// If directDrive isn't enabled but the lock is on, set it to false
		if (!directDrive && directLock) directLock = false;
		// Store the drive values
		this.left = leftSpeed;
		this.right = rightSpeed;
		// Drive the robot!
		chassis.tankDrive(leftSpeed, rightSpeed);		
	}
	

}
