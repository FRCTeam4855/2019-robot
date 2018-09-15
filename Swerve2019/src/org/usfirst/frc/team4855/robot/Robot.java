/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//yeet

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
	
	double previousError = 0;
	double integral = 0;
	
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
	PIDController pidDir[] = { // p started as .035
		new PIDController(0.035,0,0.01,encoder[0],motorDir[0]),
		new PIDController(0.035,0,0.01,encoder[1],motorDir[1]),
		new PIDController(0.035,0,0.01,encoder[2],motorDir[2]),
		new PIDController(0.035,0,0.01,encoder[3],motorDir[3])
		/*new PIDController(.3,.01,0,encoder[0],motorDir[0]),
		new PIDController(.3,.01,0,encoder[1],motorDir[1]),
		new PIDController(.3,.01,0,encoder[2],motorDir[2]),
		new PIDController(.3,.01,0,encoder[3],motorDir[3])*/
		
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
	public void teleopInit() {
		for (int i=0;i<=3;i++) {
			pidDir[i].setInputRange(-1,1);
			pidDir[i].setContinuous();
			pidDir[i].enable();
			encoder[i].reset();
		}
	}
	
	@Override
	public void teleopPeriodic() {
		double driveVal[] = {
			controlDrive.getRawAxis(1), //fwd
			controlDrive.getRawAxis(0), //str
			controlDrive.getRawAxis(4)  //rcw
		};
		for (int i=0;i<=2;i++) {
			if (driveVal[i] <= .1  && driveVal[i] >= -.1) {
				driveVal[i] = 0;
			} else {
				driveVal[i] /= 2;
			}
		}
		setPIDs(pidDir,true);
		swerveDrive(driveVal[0], driveVal[1], driveVal[2]);
	}
	
	public void swerveDrive(double fwd, double str, double rcw) {
	    // quick maffs
		str *= -1;
	    double a = fwd - rcw * (MEASURE_LENGTH / MEASURE_DIAG);
	    double b = fwd + rcw * (MEASURE_LENGTH / MEASURE_DIAG);
	    double c = str - rcw * (MEASURE_WIDTH / MEASURE_DIAG);
	    double d = str + rcw * (MEASURE_WIDTH / MEASURE_DIAG);
	    // NEW labeled as 0 back right, 1 back left, 2 front right, 3 front left
	    // OLD labeled as 0 front right, 1 front left, 2 back left, 3 back right
	    double speed[] = {
	    		Math.sqrt ((b * b) + (d * d)), // 2 new 0 old
	    		Math.sqrt ((b * b) + (c * c)), // 3 new 1 old
	    		Math.sqrt ((a * a) + (d * d)), // 0 new 2 old
	    		Math.sqrt ((a * a) + (c * c)) // 1 new 3 old
	    };
	    double angle[] = {
	    		Math.atan2 (b, d) / Math.PI, // 2 new 0 old
	    		Math.atan2 (b, c) / Math.PI, // 3 new 1 old
	    		Math.atan2 (a, d) / Math.PI, // 0 new 2 old
	    		Math.atan2 (a, c) / Math.PI // 1 new 3 old
	    };
	    
	    for (int i=0;i<=3;i++) {
		    motorDrive[i].set(speed[i]);
		    double setpoint = angle[i];
		    pidDir[i].setSetpoint(setpoint);
		    SmartDashboard.putNumber("Setpoint"+Integer.toString(i), setpoint);
		    SmartDashboard.putNumber("motorDir"+Integer.toString(i), motorDir[i].get());
		    SmartDashboard.putNumber("Encoder"+Integer.toString(i), encoder[i].get());
		    SmartDashboard.putNumber("SETPOINT"+Integer.toString(i), pidDir[i].getSetpoint());
		    //PID(.03,.01,0,setpoint,encoder[i].get(),motorDir[i]);
	    }
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		// Tuning wheels
		
		setPIDs(pidDir,false);
		int wheelTune = 0;
		if (controlDrive.getRawButton(1)) wheelTune = 0;
		if (controlDrive.getRawButton(2)) wheelTune = 1;
		if (controlDrive.getRawButton(3)) wheelTune = 2;
		if (controlDrive.getRawButton(4)) wheelTune = 3;
		
		switch (wheelTune) {
		case 0:
			if (controlDrive.getRawButton(5)) motorDir[0].set(0.3);
			else if (controlDrive.getRawButton(6)) motorDir[0].set(-0.3); else motorDir[0].set(0);
			break;
		case 1:
			if (controlDrive.getRawButton(5)) motorDir[1].set(0.3);
			else if (controlDrive.getRawButton(6)) motorDir[1].set(-0.3); else motorDir[1].set(0);
			break;
		case 2:
			if (controlDrive.getRawButton(5)) motorDir[2].set(0.3);
			else if (controlDrive.getRawButton(6)) motorDir[2].set(-0.3); else motorDir[2].set(0);
			break;
		case 3:
			if (controlDrive.getRawButton(5)) motorDir[3].set(0.3);
			else if (controlDrive.getRawButton(6)) motorDir[3].set(-0.3); else motorDir[3].set(0);
			break;
		}
	}
	
	public void setPIDs(PIDController[] pids, boolean enabled) {
		// Quick way to enable or disable a group of PID controllers
		for (int i=0;i<=3;i++) {
			pids[i].setEnabled(enabled);
		}
	}
	
	public void PID(double P, double I, double D, double setpoint, double current, Spark motorAdjust) {
		// This is a custom PID loop I tried to make based on some thing online but it doesn't really work
		
		double error = setpoint - current;
		integral += (error * .02);
		double derivative = (error - previousError) / .02;
		motorAdjust.set(P*error + I*integral + D*derivative);
		previousError = error;
	}
}