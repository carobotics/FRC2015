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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Gyro;

public class Robot extends SampleRobot {
	Preferences prefs;

    RobotDrive drive = new RobotDrive(0, 1, 2, 3);
    CANTalon chainMotor = new CANTalon(0);
    CANTalon chainMotorSlave = new CANTalon(1);

    Joystick rightStick = new Joystick(0);
    Joystick leftStick = new Joystick(1);
    JoystickButton halfSpeedButton = new JoystickButton(leftStick, 1);
    JoystickButton calibrateButton = new JoystickButton(rightStick, 11);
    JoystickButton minButton = new JoystickButton(rightStick, 4);
    JoystickButton maxButton = new JoystickButton(rightStick, 6);
    JoystickButton lowButton = new JoystickButton(rightStick, 3);
    JoystickButton feedButton = new JoystickButton(rightStick, 5);

    CameraServer cameraServer;
    Gyro gyro;
    ControlMode mode;
    SendableChooser autoChooser;

    double ticksPerInch = 136.3158;
    double hookMoveAmount = 20;
    double desiredChainPosition = 0.0;
    double chainStartingPosition = 0.0;
    double chainSpeed = 170.0;
    int prevDir = 0;

    long lastDashUpdateTime = 0;

    double drivingMult = 0.5;
    double turningMult = 0.3;
    double xyClipAmt = 0.2;
    double zClipAmt = 0.2;
    double brakeYMult = 0.3;
    double brakeZMult = 0.7;

    double pP = 5.0;
    double pI = 0.0;
    double pD = 30.0;

    double vP = 2.0;
    double vI = 0.001;
    double vD = 0.0;
    
    public void print(String msg) {
        System.out.println(msg);
    }
    
    public void robotInit() {
    	prefs = Preferences.getInstance();
    	gyro = new Gyro(0);
    	gyro.initGyro();
    	
    	chainMotorSlave.changeControlMode(ControlMode.Follower);
    	chainMotorSlave.set(0); //set to follow chainMotor, with id 0
    	chainMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);

    	autoChooser = new SendableChooser();
    	autoChooser.addDefault("Do nothing", "nothing");
    	autoChooser.addObject("Take bin and tote", "bin&tote");
    	SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

    	if (!prefs.containsKey("3chainCalibration")) {
    		savePositionAsCalibration();
    	} else {
	    	chainStartingPosition = prefs.getDouble("3chainCalibration", 0.0);
    	}

    	if (!prefs.containsKey("3lastRawChainPosition")) {
    		saveLastRawPosition();
    	} else if (chainMotor.getPosition() == 0) {
	    	chainStartingPosition -= prefs.getDouble("3lastRawChainPosition", 0.0);
	    	prefs.putDouble("3chainCalibration", chainStartingPosition);
    	} else {
    		print("Code restarted, but encoder values were not lost: " + chainMotor.getPosition());
    	}

    	if (!prefs.containsKey("turnAngle")) {
    		prefs.putDouble("turnAngle", 81);
    	}
    	if (!prefs.containsKey("robotMoveForwardTime")) {
    		prefs.putLong("robotMoveForwardTime", 500);
    	}
    	if (!prefs.containsKey("robotMoveForwardTime2")) {
    		prefs.putLong("robotMoveForwardTime2", 200);
    	}
    	if (!prefs.containsKey("robotMoveForwardTime3")) {
    		prefs.putLong("robotMoveForwardTime3", 2000);
    	}

    	desiredChainPosition = getChainPosition();
    	positionControlMode();

        //initialize SmartDashboard with default values
        SmartDashboard.putNumber("DrivingMultiplier", drivingMult);
        SmartDashboard.putNumber("TurningMultiplier", turningMult);
        SmartDashboard.putNumber("xyClipAmt", xyClipAmt);
        SmartDashboard.putNumber("zClipAmt", zClipAmt);
        SmartDashboard.putNumber("brakeYMult", brakeYMult);
        SmartDashboard.putNumber("brakeZMult", brakeZMult);
        SmartDashboard.putNumber("chainSpeed", chainSpeed);
        SmartDashboard.putNumber("pP", pP);
        SmartDashboard.putNumber("pI", pI);
        SmartDashboard.putNumber("pD", pD);
        SmartDashboard.putNumber("vP", vP);
        SmartDashboard.putNumber("vI", vI);
        SmartDashboard.putNumber("vD", vD);
        
        drive.setInvertedMotor(MotorType.kFrontRight, true);
        drive.setInvertedMotor(MotorType.kRearRight, true);
        cameraServer = CameraServer.getInstance();
        cameraServer.setQuality(10);
        cameraServer.startAutomaticCapture("cam0");
    }

    public void disabled() {
        prefs.save();
        print("Preferences saved.");
    }
 
    public void autonomous() {
        if (isAutonomous() && isEnabled() && ((String) autoChooser.getSelected()).equals("bin&tote")) {
            double startPos = getChainPosition();
    		velocityControlMode();
    		int ultimateHeight = 33;
            while (getChainPosition() < startPos + 8 && isAutonomous() && isEnabled()) {
            	chainMotor.set(chainSpeed);
            }
            long startTime = System.currentTimeMillis();
            long duration = prefs.getLong("robotMoveForwardTime", 500);
            while (System.currentTimeMillis() <= startTime + duration) {
                drive.mecanumDrive_Cartesian(0, -0.3, 0, 0); //drive straight
                if (getChainPosition() < startPos + ultimateHeight)
                	chainMotor.set(chainSpeed);
            }
            drive.mecanumDrive_Cartesian(0, 0, 0, 0); //stop
            while (getChainPosition() < startPos + ultimateHeight && isAutonomous() && isEnabled()) {
            	chainMotor.set(chainSpeed);
            }
        	chainMotor.set(0);

        	//turn left
        	double startAngle = gyro.getAngle();
    		print("Start Angle: " + startAngle);
        	while (gyro.getAngle() > startAngle - prefs.getDouble("turnAngle", 81) && isAutonomous() && isEnabled()) {
        		print("" + gyro.getAngle());
                drive.mecanumDrive_Cartesian(0, 0, -0.3, 0); //turn left
        	}

        	//drive to bin
            startTime = System.currentTimeMillis();
            duration = prefs.getLong("robotMoveForwardTime2", 200);
            while (System.currentTimeMillis() <= startTime + duration) {
                drive.mecanumDrive_Cartesian(0, -0.25, 0, 0); //drive straight
            }
            drive.mecanumDrive_Cartesian(0, 0, 0, 0); //stop

            //lift bin
            startPos = getChainPosition();
            while (getChainPosition() < startPos + 10 && isAutonomous() && isEnabled()) {
            	chainMotor.set(chainSpeed);
            }
            chainMotor.set(0);

        	//turn right
        	while (gyro.getAngle() < startAngle && isAutonomous() && isEnabled()) {
        		print("" + gyro.getAngle());
                drive.mecanumDrive_Cartesian(0, 0, 0.3, 0); //turn right
        	}

        	//drive to zone
            startTime = System.currentTimeMillis();
            duration = prefs.getLong("robotMoveForwardTime3", 2000);
            while (System.currentTimeMillis() <= startTime + duration) {
                drive.mecanumDrive_Cartesian(0, -0.3, 0, 0); //drive straight
            }
            drive.mecanumDrive_Cartesian(0, 0, 0, 0); //stop

            desiredChainPosition = getChainPosition();
        }
    }
 
 	/** Main operator control. */
    public void operatorControl() {
        positionControlMode();
        while (isOperatorControl() && isEnabled()) {
            long curTime = System.currentTimeMillis();
            
            //update smart dashboard values
            if (curTime > lastDashUpdateTime + 1000) {
                drivingMult = SmartDashboard.getNumber("DrivingMultiplier");
                turningMult = SmartDashboard.getNumber("TurningMultiplier");
                xyClipAmt = SmartDashboard.getNumber("xyClipAmt");
                zClipAmt = SmartDashboard.getNumber("zClipAmt");
                brakeYMult = SmartDashboard.getNumber("brakeYMult");
                brakeZMult = SmartDashboard.getNumber("brakeZMult");
                chainSpeed = SmartDashboard.getNumber("chainSpeed");
                pP = SmartDashboard.getNumber("pP");
                pI = SmartDashboard.getNumber("pI");
                pD = SmartDashboard.getNumber("pD");
                vP = SmartDashboard.getNumber("vP");
                vI = SmartDashboard.getNumber("vI");
                vD = SmartDashboard.getNumber("vD");
                if (mode == ControlMode.Position) {
	                if (pP != chainMotor.getP() || pI != chainMotor.getI() || pD != chainMotor.getD())
	                	chainMotor.setPID(pP, pI, pD);
	            } else if (mode == ControlMode.Speed) {
	                if (vP != chainMotor.getP() || vI != chainMotor.getI() || vD != chainMotor.getD())
	                	chainMotor.setPID(vP, vI, vD);
	            }
                SmartDashboard.putNumber("Encoder position", getChainPosition());
                SmartDashboard.putNumber("Desired position", desiredChainPosition);
                SmartDashboard.putNumber("Raw position", chainMotor.getPosition());

                saveLastRawPosition();

                lastDashUpdateTime = curTime;
            }

            //check if we should calibrate
            if (calibrateButton.get())
            	savePositionAsCalibration();
            
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

            //main drive
            if (pov != -1) { //debug driving with the POV
            	drive.mecanumDrive_Polar(debugSpeed, pov, 0);
            } else { //normal driving
	            drive.mecanumDrive_Cartesian(xVal * drivingMult, yVal * drivingMult, zVal * turningMult, 0);
            }

            //chain driving
            double chainPov = rightStick.getPOV(0);
        	double dif = getChainPosition() - desiredChainPosition;
        	int dir = dif == 0 ? 0 : (int) (-dif / Math.abs(dif));
        	double chainJoyVal = clip(-rightStick.getY() * 0.5, 0.1);

        	if (minButton.get() || maxButton.get() || lowButton.get() || feedButton.get()) {
        		if (minButton.get())
        			desiredChainPosition = 0;
        		if (maxButton.get())
        			desiredChainPosition = 60;
        		if (lowButton.get())
        			desiredChainPosition = 16;
        		if (feedButton.get())
        			desiredChainPosition = 30;
        		velocityControlMode();
        		prevDir = 0;
        	}
        	if (chainJoyVal == 0) {
        		if (mode == ControlMode.PercentVbus) {
        			positionControlMode();
	        		desiredChainPosition = getChainPosition();
        		}

	            if (mode == ControlMode.Speed) {
	            	if (dir == -prevDir)
	            		positionControlMode();
	            	else {
	            		if (chainPov == 0 && dir == -1) {
	            			desiredChainPosition += hookMoveAmount;
	            			dir *= -1;
	            		}
	            		else if (chainPov == 180 && dir == 1) {
	            			desiredChainPosition -= hookMoveAmount;
	            			dir *= -1;
	            		}
	            		
		            	chainMotor.set(chainSpeed * dir);
	            	}
            		prevDir = dir;
	            } else { //mode == ControlMode.Position
	            	setChainPosition(desiredChainPosition);

	            	if (chainPov == 0 || chainPov == 180) {
	            		if (chainPov == 180 && desiredChainPosition % (int) hookMoveAmount == 0)
	            			desiredChainPosition -= hookMoveAmount;
	            		else {
	            			desiredChainPosition = ((int) desiredChainPosition / (int) hookMoveAmount) * hookMoveAmount;
	            			if (chainPov == 0)
	            				desiredChainPosition += hookMoveAmount;
	            		}
	            		velocityControlMode();
	            	}
	            	prevDir = 0;
	            }
	        } else {
            	if (mode != ControlMode.PercentVbus)
            		percentControlMode();

        		chainMotor.set(chainJoyVal);
	        }
            
            Timer.delay(0.005); //do not delete
        }
    }

    /** This saves the current raw position as the last known raw position. */
    private void saveLastRawPosition() {
		prefs.putDouble("2lastRawChainPosition", chainMotor.getPosition());
    }

    /** This saves the current raw position as the 0.0 position. */
    private void savePositionAsCalibration() {
    	chainStartingPosition = chainMotor.getPosition();
		prefs.putDouble("2chainCalibration", chainStartingPosition);
    	desiredChainPosition = getChainPosition();
    }
    
    /** Change control mode to percent voltage. */
    private void percentControlMode() {
    	mode = ControlMode.PercentVbus;
    	chainMotor.changeControlMode(mode);
    }

    /** Change control mode to position. */
    private void positionControlMode() {
    	mode = ControlMode.Position;
    	chainMotor.changeControlMode(mode);
    	chainMotor.setPID(pP, pI, pD);
    }

    /** Change control mode to velocity. */
    private void velocityControlMode() {
    	mode = ControlMode.Speed;
    	chainMotor.changeControlMode(mode);
    	chainMotor.setPID(vP, vI, vD);
    }

    /** This method returns the accurate chain position based on the calibration. */
    private double getChainPosition() {
    	return (chainMotor.getPosition() - chainStartingPosition) / ticksPerInch;
    }

    /** This method sets the chain position based on the calibration. */
    private void setChainPosition(double pos) {
    	chainMotor.set(pos * ticksPerInch + chainStartingPosition);
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
