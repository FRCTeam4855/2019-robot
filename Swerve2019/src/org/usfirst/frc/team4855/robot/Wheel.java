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
	    	angle += 180;
	    	if (angle < 180) {
	    		angle -= 360;
	    	}
	    }
	    
	    motorDrive.set(speed * flip);
	    double setpoint = angle;
	    pid.setEnabled(true);
	    pid.setSetpoint(setpoint);
	}
}
