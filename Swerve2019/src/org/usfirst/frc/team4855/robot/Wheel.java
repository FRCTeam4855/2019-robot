package org.usfirst.frc.team4855.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;

public class Wheel {
	int id;				// local identification number
	Encoder encoder;	// local encoder
	Spark motorDir;		// local direction motor
	Spark motorDrive;	// local drive motor
	PIDController pid;  // local PID for direction
	
	//NAVX CONSTRUCTOR
	AHRS ahrs = Robot.ahrs;
	
	// Defining swerve vars
	double a, b, c, d, max, temp, rads;
	double angleLast, angleCalc, addto, flipVal;
	
	// This command executes all of swerve's controllers and unique IDs
	public Wheel(int d, Encoder e, Spark f, Spark g, PIDController h) {
		id = d;encoder = e;motorDir = f;
		motorDrive = g;pid = h;
	}
	double flip = 1;
	
	
	
	public void swerve(double str, double fwd, double rcw) {
		double speed = 0, angle = 0;
		
		if (Robot.driverOriented) {
			rads = ahrs.getYaw() * Math.PI/180;
			temp = fwd*Math.cos(rads) + str*Math.sin(rads);
			str = -fwd*Math.sin(rads) + str*Math.cos(rads);
			fwd = temp;
		}
		
		a = fwd - rcw * (Robot.MEASURE_LENGTH / Robot.MEASURE_DIAG);
	    b = fwd + rcw * (Robot.MEASURE_LENGTH / Robot.MEASURE_DIAG);
	    c = str - rcw * (Robot.MEASURE_WIDTH / Robot.MEASURE_DIAG);
	    d = str + rcw * (Robot.MEASURE_WIDTH / Robot.MEASURE_DIAG);
	    
	    switch (id) {
		    case 0:
		    	speed = Math.sqrt ((b * b) + (d * d));
		    	angle = findWheelAngle(b,d);
		    	break;
		    case 1:
		    	speed = Math.sqrt ((b * b) + (c * c));
		    	angle = findWheelAngle(b,c);
		    	break;
		    case 2:
		    	speed = Math.sqrt ((a * a) + (c * c));
		    	angle = findWheelAngle(a,c);
		    	break;
		    case 3:
		    	speed = Math.sqrt ((a * a) + (d * d));
		    	angle = findWheelAngle(a,d);
		    	break;
	    }
	    motorDrive.set(speed * getFlip());
	    double setpoint = angle;
	    pid.setEnabled(true);
	    pid.setSetpoint(setpoint);
	}
	
	private double findWheelAngle(double m1, double m2) {
		double angleCalc = (Math.atan2(m1, m2) * 180 / Math.PI) * Robot.ETD;
		if (angleCalc == 0) {
			// If the calculated angle is 0, set the angle to whatever angle was during last step
			angleCalc = angleLast;
		}
		angleCalc += addto;	// I wish I knew what this means
		angleCalc += flipVal;	// Compensates for flipping if needed
		if (Math.abs(encoder.get()-angleCalc) > 90*Robot.ETD && Math.abs(encoder.get()-angleCalc) < 270*Robot.ETD) {
			// If the wheel has to turn more than 90 degrees and less than 270, flip the wheel
			angleCalc -= flipVal;
			if (flipVal == 0) flipVal = 180 * Robot.ETD; else flipVal = 0;
			angleCalc += flipVal;
		}
		
		if (angleLast - angleCalc > 185*Robot.ETD) {addto += 360*Robot.ETD;angleCalc += 360*Robot.ETD;} //For magnetic encoders, USE 412 (ABS 4048)
		if (angleLast - angleCalc < -185*Robot.ETD) {addto -= 360*Robot.ETD;angleCalc -= 360*Robot.ETD;}
		if (encoder.get() - angleCalc > 380*Robot.ETD) {addto += 360*Robot.ETD;angleCalc += 360*Robot.ETD;}
		if (encoder.get() - angleCalc < -380*Robot.ETD) {addto -= 360*Robot.ETD;angleCalc -= 360*Robot.ETD;}
		
		angleLast = angleCalc;
		return angleCalc;
		
	}	// End of findWheelAngle
	
	public int getFlip() {
		if (flipVal == 0) return 1; else return -1;
	}
	
	public void reset() {
		flipVal = 0;addto = 0;
		angleLast = 0;angleCalc = 0;
		pid.setSetpoint(0);
	}
}
