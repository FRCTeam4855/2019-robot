/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4855.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	
	Joystick controlDrive = new Joystick(0);
	
	public final double MEASURE_WIDTH = 33;
	public final double MEASURE_LENGTH = 28;
	public final double MEASURE_DIAG = Math.sqrt ((MEASURE_LENGTH * MEASURE_LENGTH) + (MEASURE_WIDTH * MEASURE_WIDTH));
	public final double MEASURE_MAXVOLTS = 4.95;
	
	// ALL MOTORS
	
	//Directional motors
	Spark motorDir[] = {
		new Spark(7),
		new Spark(6),
		new Spark(4),
		new Spark(5)
	};
	
	//Movement motors
	Spark motorDrive[] = {
		new Spark(1),
		new Spark(2),
		new Spark(10),
		new Spark(0)
	};
	
	// Magnetic encoders
	Encoder encoder[] = {
		new Encoder(2,3),
		new Encoder(6,7),
		new Encoder(4,5),
		new Encoder(0,1)
	};
	
	// PID Loops for the rotational motors
	PIDController pidDir[] = {
		/*new PIDController(0.035,0,0.01,encoder[0],motorDir[0]),
		new PIDController(0.035,0,0.01,encoder[1],motorDir[1]),
		new PIDController(0.035,0,0.01,encoder[2],motorDir[2]),
		new PIDController(0.035,0,0.01,encoder[3],motorDir[3])*/
		new PIDController(1,0,0,encoder[0],motorDir[0]),
		new PIDController(1,0,0,encoder[1],motorDir[1]),
		new PIDController(1,0,0,encoder[2],motorDir[2]),
		new PIDController(1,0,0,encoder[3],motorDir[3])
		
	};
	
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		for (int i=0;i<=3;i++) {
			pidDir[i].setInputRange(-1,1);
			pidDir[i].setContinuous();
			pidDir[i].enable();
		}
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
		
	}

	@Override
	public void teleopPeriodic() {
		swerveDrive(controlDrive.getRawAxis(1), controlDrive.getRawAxis(0), controlDrive.getRawAxis(4));
	}
	
	public void swerveDrive(double x1, double y1, double x2) {
	    // The following operations do funny math things
		y1 *= -1;
	    double a = x1 - x2 * (MEASURE_LENGTH / MEASURE_DIAG);
	    double b = x1 + x2 * (MEASURE_LENGTH / MEASURE_DIAG);
	    double c = y1 - x2 * (MEASURE_WIDTH / MEASURE_DIAG);
	    double d = y1 + x2 * (MEASURE_WIDTH / MEASURE_DIAG);
	    
	    double speed[] = {
	    		Math.sqrt ((a * a) + (d * d)), // labeled as back right, back left, front right, front left
	    		Math.sqrt ((a * a) + (c * c)),
	    		Math.sqrt ((b * b) + (d * d)),
	    		Math.sqrt ((b * b) + (c * c))
	    };
	    double angle[] = {
	    		Math.atan2 (a, d) / Math.PI, // labeled as back right, back left, front right, front left
	    		Math.atan2 (a, c) / Math.PI,
	    		Math.atan2 (b, d) / Math.PI,
	    		Math.atan2 (b, c) / Math.PI
	    };
	    
	    for (int i=0;i<=3;i++) {
		    motorDrive[i].set(speed[i]);
	
		    double setpoint = angle[i] * (MEASURE_MAXVOLTS * 0.5) + (MEASURE_MAXVOLTS * 0.5); // Optimization offset can be calculated here.
		    if (setpoint < 0) {
		        setpoint = MEASURE_MAXVOLTS + setpoint;
		    }
		    if (setpoint > MEASURE_MAXVOLTS) {
		        setpoint = setpoint - MEASURE_MAXVOLTS;
		    }
	
		    pidDir[i].setSetpoint(setpoint);
	    }
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}