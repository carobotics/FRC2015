/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5160.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;

public class Robot extends SampleRobot {
    RobotDrive drive = new RobotDrive(0, 1, 2, 3);
    CANTalon chainMotor = new CANTalon(0);
    CANTalon chainMotorSlave = new CANTalon(1);

    Joystick rightStick = new Joystick(0);
    Joystick leftStick = new Joystick(1);
    JoystickButton halfSpeedButton = new JoystickButton(leftStick, 1);
    JoystickButton calibrate = new JoystickButton(leftStick, 11);
    JoystickButton gyroToggle = new JoystickButton(leftStick, 12);

    CameraServer cameraServer;
    Gyro gyro;
    double xAxis = 0.0;
    double yAxis = 0.0;
    double zAxis = 0.0;

    long lastDashUpdateTime = 0;

    boolean gyroButtonPreviouslyPressed = false;
    boolean gyroEnabled = false;
    boolean gyroWasInitialized = false;
    double drivingMult = 0.5;
    double turningMult = 0.3;
    double plusAccelVal = 0.05;
    double negAccelVal = 0.05;
    double xyClipAmt = 0.2;
    double zClipAmt = 0.2;
    double brakeYMult = 0.3;
    double brakeZMult = 0.7;
    
    public void print(String msg) {
        System.out.println(msg);
    }
    
    public void robotInit() {
    	chainMotorSlave.changeControlMode(ControlMode.Follower);
    	chainMotorSlave.set(0); //set to follow chainMotor, with id 0
    	chainMotor.changeControlMode(ControlMode.PercentVbus); //Need to change to Position once we get encoder.

        //initialize SmartDashboard with default values
        SmartDashboard.putNumber("DrivingMultiplier", drivingMult);
        SmartDashboard.putNumber("TurningMultiplier", turningMult);
        SmartDashboard.putNumber("+DriveAcceleration", plusAccelVal);
        SmartDashboard.putNumber("-DriveAcceleration", negAccelVal);
        SmartDashboard.putNumber("xyClipAmt", xyClipAmt);
        SmartDashboard.putNumber("zClipAmt", zClipAmt);
        SmartDashboard.putNumber("brakeYMult", brakeYMult);
        SmartDashboard.putNumber("brakeZMult", brakeZMult);
        
        drive.setInvertedMotor(MotorType.kFrontRight, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
        cameraServer = CameraServer.getInstance();
        cameraServer.setQuality(10);
        cameraServer.startAutomaticCapture("cam0");
        gyro = new Gyro(0);
    }
 
    public void autonomous() {
        if (isAutonomous() && isEnabled()) {
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() < startTime + 2000) {
                drive.mecanumDrive_Cartesian(0, -1, 0, 0); //drive straight
            }
            drive.mecanumDrive_Cartesian(0, 0, 0, 0);
        }
    }
 
 	/** Main operator control. */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            long curTime = System.currentTimeMillis();
            
            //update smart dashboard values
            if (curTime > lastDashUpdateTime + 1000) {
                drivingMult = SmartDashboard.getNumber("DrivingMultiplier");
                turningMult = SmartDashboard.getNumber("TurningMultiplier");
                plusAccelVal = SmartDashboard.getNumber("+DriveAcceleration");
                negAccelVal = SmartDashboard.getNumber("-DriveAcceleration");
                xyClipAmt = SmartDashboard.getNumber("xyClipAmt");
                zClipAmt = SmartDashboard.getNumber("zClipAmt");
                brakeYMult = SmartDashboard.getNumber("brakeYMult");
                brakeZMult = SmartDashboard.getNumber("brakeZMult");

                lastDashUpdateTime = curTime;
            }

            //gyro calibration
            if (calibrate.get() && gyroEnabled)
            	gyro.reset();

            //gyro toggle
            if (gyroToggle.get() && !gyroButtonPreviouslyPressed) {
            	gyro.reset();
            	gyroEnabled = !gyroEnabled;
            	gyroButtonPreviouslyPressed = true;
            	if (!gyroWasInitialized) {
        			gyro.initGyro();
        			gyroWasInitialized = true;
            	}
            } else {
            	gyroButtonPreviouslyPressed = false;
            }

            //get gyro value
            double angle = 0;
            if (gyroEnabled)
            	angle = gyro.getAngle();
            
            //get joystick values
            double xVal = clip(leftStick.getX(), xyClipAmt);
            double yVal = clip(leftStick.getY(), xyClipAmt);
            double zVal = clip(leftStick.getZ(), zClipAmt);

            //decrease values if brake pressed
            if (halfSpeedButton.get()) {
            	yVal *= brakeYMult;
            	zVal *= brakeZMult;
            }

            //drive debugging
            double debugSpeed = (-leftStick.getThrottle() + 1) / 2;
            double pov = leftStick.getPOV(0);

            if (pov != -1) { //debug driving with the POV
            	drive.mecanumDrive_Polar(debugSpeed, pov, 0);
            } else { //normal driving
	            drive.mecanumDrive_Cartesian(xVal * drivingMult, yVal * drivingMult, zVal * turningMult, angle);
            }

            //chain driving
            chainMotor.set(-rightStick.getY());
            
            Timer.delay(0.005); //do not delete
        }
    }

    /** If val is less than clipAmt away from 0, then 0 is returned.
     *  Otherwise, the val will be scaled from the clipAmt to 1.0 or -1.0 */
    private double clip(double val, double clipAmt) {
        if (Math.abs(val) < clipAmt) {
            return 0;
        } else {
        	if (val > 0)
        		val -= clipAmt;
        	else
        		val += clipAmt;
        	val *= 1 / (1 - clipAmt);
        	return val;
        }
    }
}
