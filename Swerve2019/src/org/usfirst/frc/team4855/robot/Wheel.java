package org.usfirst.frc.team4855.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;

public class Wheel {
	int id;				// local identification number
	Encoder encoder;	// local encoder
	Spark motorDir;		// local direction motor
	Spark motorDrive;	// local drive motor
	PIDController pid;  // local PID for direction
	// Defining swerve vars
	double a, b, c, d, max, temp, rads;
	
	// This command executes all of swerve's controllers and unique IDs
	public Wheel(int d, Encoder e, Spark f, Spark g, PIDController h) {
		id = d;encoder = e;motorDir = f;
		motorDrive = g;pid = h;
	}
	double flip = 1;
	
	public void swerve(double str, double fwd, double rcw, boolean input) {
		double speed = 0, angle = 0;
		
		a = fwd - rcw * (Robot.MEASURE_LENGTH / Robot.MEASURE_DIAG);
	    b = fwd + rcw * (Robot.MEASURE_LENGTH / Robot.MEASURE_DIAG);
	    c = str - rcw * (Robot.MEASURE_WIDTH / Robot.MEASURE_DIAG);
	    d = str + rcw * (Robot.MEASURE_WIDTH / Robot.MEASURE_DIAG);
	    
	    switch (id) {
		    case 0:
		    	speed = Math.sqrt ((b * b) + (d * d));
		    	angle = (Math.atan2(b, d) * 180 / Math.PI) * Robot.ETD;
		    	break;
		    case 1:
		    	speed = Math.sqrt ((b * b) + (c * c));
		    	angle = (Math.atan2(b, c) * 180 / Math.PI) * Robot.ETD;
		    	break;
		    case 2:
		    	speed = Math.sqrt ((a * a) + (c * c));
		    	angle = (Math.atan2(a, c) * 180 / Math.PI) * Robot.ETD;
		    	break;
		    case 3:
		    	speed = Math.sqrt ((a * a) + (d * d));
		    	angle = (Math.atan2(a, d) * 180 / Math.PI) * Robot.ETD;
		    	break;
	    }
	    
	    // Adjustments and other calculations here
	    
	    if (input == false) {
	    	// Input has just begun, wheels should flip if directional requirements are met
	    	if (fwd < 0) {
	    		flip = -1;
	    	}
	    }
	    
	    if (flip == -1) {
	    	angle += 180 * Robot.ETD;
	    	if (angle < 180 * Robot.ETD) {
	    		angle -= 360 * Robot.ETD;
	    	}
	    }
	    
		/*
		When it comes to determining whether or not to flip the wheel direction,
		the `input` var probably won't help you much. What you need to do
		is determine how far the wheel needs to rotate from its CURRENT
		position. If the wheel will need to rotate more than 90 degrees (and less 
		than 270 degrees), that's when you'll want to apply a wheel speed flip. 
		The SwerveWheel object from the 2018 code demonstrates this on lines 19-24.
		   
		Also, your backwards code didn't work because you were adding 180 to
		your `angle` var, which you weren't converting to encoder units.
		*/
		
	    motorDrive.set(speed * flip);
	    double setpoint = angle;
	    pid.setEnabled(true);
	    pid.setSetpoint(setpoint);
	}
}
