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
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.vision.AxisCamera;
// import edu.wpi.first.wpilibj.CameraServer;
// import edu.wpi.first.wpilibj.Gyro;
import java.util.Date;

public class Robot extends SampleRobot {
 
    RobotDrive drive = new RobotDrive(0, 1, 2, 3);
    Joystick rightStick = new Joystick(0);
    Joystick leftStick = new Joystick(1);
    JoystickButton calibrate = new JoystickButton(rightStick, 6);
    // AxisCamera camera;
    // CameraServer cameraServer;
    // Gyro gyro;
    double xAxis = 0.0;
    double yAxis = 0.0;

    long dashValUpdateTime = 0;

    double drivingMult = 1.0;
    double turningMult = 1.0;
    double plusAccelVal = 0.05;
    double negAccelVal = 0.05;
    double xyClipAmt = 0.1;
    double zClipAmt = 0.1;
    
    public void print(String msg) {
        System.out.println(msg);
    }
    
    public void robotInit() {
        // camera = new AxisCamera("camera ip, check webdash");
        // cameraServer = CameraServer.getInstance();
        // cameraServer.startAutomaticCapture("camera name, like cam1");
        // gyro = new Gyro(0);
        // gyro.initGyro();

        //initialize SmartDashboard values
        SmartDashboard.putNumber("DrivingMultiplier", drivingMult);
        SmartDashboard.putNumber("TurningMultiplier", turningMult);
        SmartDashboard.putNumber("+DriveAcceleration", plusAccelVal);
        SmartDashboard.putNumber("-DriveAcceleration", negAccelVal);
        SmartDashboard.putNumber("xyClipAmt", xyClipAmt);
        SmartDashboard.putNumber("zClipAmt", zClipAmt);
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
 
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            long curTime = new Date().getTime();
            
            //smart dashboard values
            if (curTime > dashValUpdateTime + 1000) {
                drivingMult = SmartDashboard.getNumber("DrivingMultiplier");
                turningMult = SmartDashboard.getNumber("TurningMultiplier");
                plusAccelVal = SmartDashboard.getNumber("+DriveAcceleration");
                negAccelVal = SmartDashboard.getNumber("-DriveAcceleration");
                xyClipAmt = SmartDashboard.getNumber("xyClipAmt");
                zClipAmt = SmartDashboard.getNumber("zClipAmt");
            } else {
                dashValUpdateTime = curTime;
            }

            //gyro calibration
            // if (calibrate.get())
            // 	gyro.reset();
            
            //START: Main Driving
            
            //get joystick values
            double xVal = rightStick.getX();
            double yVal = rightStick.getY();
            double zVal = rightStick.getZ();

            //clip x, y, and z values if they are close to center
            if (Math.abs(xVal) < xyClipAmt)
                xVal = 0;
            if (Math.abs(yVal) < xyClipAmt)
                yVal = 0;
            if (Math.abs(zVal) < zClipAmt)
                zVal = 0;
            
            //drive smoothing with limited acceleration
            double xDif = xVal - xAxis;
            double yDif = yVal - yAxis;
 
            double accelVal;
            if (xDif < 0 && xAxis > 0 || xDif > 0 && xAxis < 0)
                accelVal = negAccelVal;
            else
                accelVal = plusAccelVal;
            if (Math.abs(xDif) > accelVal)
                xAxis += accelVal * (int) (xDif / Math.abs(xDif));
            else
            	xAxis = xVal;
 
            if (yDif < 0 && yAxis > 0 || yDif > 0 && yAxis < 0)
                accelVal = negAccelVal;
            else
                accelVal = plusAccelVal;
            if (Math.abs(yDif) > accelVal)
                yAxis += accelVal * (int) (yDif / Math.abs(yDif));
            else
            	yAxis = yVal;

            drive.mecanumDrive_Cartesian(xAxis * drivingMult, yAxis * drivingMult, -zVal * turningMult, 0);//gyro.getAngle();
            //END: Main Driving
            
            Timer.delay(0.005); //do not delete
        }
    }
}
