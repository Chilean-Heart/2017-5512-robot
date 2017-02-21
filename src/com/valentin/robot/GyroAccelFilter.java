package com.valentin.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;

public class GyroAccelFilter {
	
	BuiltInAccelerometer accel;
	ADXRS450_Gyro gyro;
	Range accelRange;
	double theta_g, theta_a, prev_time, theta, alpha;
	private Thread m_task;
	
	private static class FilterTask implements Runnable {
		private GyroAccelFilter imu;
		
		public FilterTask(GyroAccelFilter imu) {
			this.imu = imu;
		}

		@Override
		public void run() {
			while (true) {
				imu.calculate();
				try {
					Thread.sleep(20);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
	
	public GyroAccelFilter() {
		this(Range.k2G, 0.95);
	}
	
	public GyroAccelFilter(Range accelRange) {
		this(accelRange, 0.95);
	}
	
	public GyroAccelFilter(double k) {
		this(Range.k2G, k);
	}
	
	public GyroAccelFilter(Range accelRange, double k) {
		accel = new BuiltInAccelerometer(accelRange);
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		
		alpha = k;
		theta = gyro.getAngle();
		theta_a = 0;
		theta_g = 0;
		this.accelRange = accelRange;
		
		m_task = new Thread(new FilterTask(this));
		m_task.setDaemon(true);
		m_task.start();
		
		prev_time = Timer.getFPGATimestamp();	
	}
	
	public void calculate() {
		
		double dt;
		double sample_time = Timer.getFPGATimestamp();
		
		synchronized (this) {
			dt = sample_time - prev_time;
	    	prev_time = sample_time;
		}
    	
		double a_x = normalizeAccel(accel.getX(), this.accelRange);
		double a_y = normalizeAccel(accel.getY(), this.accelRange);
		
		theta_a = Math.atan2(a_y, a_x);
		theta_g += gyro.getRate() * dt;
		
		theta = alpha * theta_g + (1 - alpha) * theta_a;
	}
	
	private double normalizeAccel(double axisAccel, Range range) {
		switch (range) {
		case k2G:
			return axisAccel / 2;
		case k4G:
			return axisAccel / 4;
		case k8G:
			return axisAccel / 8;
		case k16G:
			return axisAccel / 16;
		default:
			throw new IllegalArgumentException("Wrong range!");
		}
	}
	
	public synchronized double getAngle() {
		return this.theta;
	}
	
	public synchronized double getRawAngle() {
		return this.gyro.getAngle();
	}

}
