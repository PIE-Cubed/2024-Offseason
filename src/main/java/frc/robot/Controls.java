// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



/****************************************************************************************** 
*
*    IMPORTANT NOTE!!!
* 
******************************************************************************************/
/***
 * The Xbox controller DPAD is known as "POV", and the input is retrieved from the getPOV function.
 * The getPOV function returns an integer from -1 to 359, going clockwise.
 * The button assignments are as follows:
 * -1 = nothing pressed
 * 0 = top
 * 45 = top right
 * 90 = right
 * 135 = bottom right
 * 180 = bottom
 * 225 = bottom left
 * 270 = left
 * 315 = top left
 */

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Start of the Controls class
 */
public class Controls {
	// CONSTANTS
	private final int DRIVE_ID = 0;
	private final int MANIPULATOR_ID = 1;

	// Controller object declaration
	private ZorroController driveController;
	private XboxController manipulatorController;

	private boolean fieldDrive = true;

	// Rate limiters
	//private SlewRateLimiter xLimiter;
	//private SlewRateLimiter yLimiter;
	//private SlewRateLimiter rotateLimiter;

	/**
	 * The constructor for the Controls class
	 */
	public Controls() {
        //System.out.println("[INFO] >> Initializing controllers...");

		// Instance Creation
		//driveController = new XboxController(DRIVE_ID);
		driveController = new ZorroController(DRIVE_ID);
		manipulatorController = new XboxController(MANIPULATOR_ID);

		fieldDrive = false;

		// Create the rate limiters
		//xLimiter      = new SlewRateLimiter(12); // -6 to 6 in two seconds
		//yLimiter      = new SlewRateLimiter(12); // -6 to 6 in two seconds
		//rotateLimiter = new SlewRateLimiter(6 * Math.PI);
	}



	/****************************************************************************************** 
    *
    *    DRIVE FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Gets the forward speed
	 * <p>Forward is positive to match chassis speed standards
	 * <p>This measures rotatation around the Y axis, which is effectively translation on the X axis
	 * 
	 * @return forwardSpeed
	 */
	public double getForwardSpeed() {
		double speed;
		double power = driveController.getLeftY();
		
		// Turns the power into a speed
		if(enablePrecisionDrive()){
			power = Math.pow(power, 3);
			speed = power * Drive.POWER_SPEED_RATIO_MPS_PREICISION;	// Use precision mode curve
		} else {
			power = Math.pow(power, 5);
			speed = power * Drive.POWER_SPEED_RATIO_MPS;	// Use regular curve
		}

		// Limits the acceleration when under driver control
		//speed = xLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the strafe speed
	 * <p>Left is negative, converted to chassis speed standards in Drive.teleopDrive()
	 * <p>This measures rotatation around the X axis, which is effectively translation on the Y axis
	 * 
	 * @return strafeSpeed
	 */
	public double getStrafeSpeed() {
		double speed;
		double power = driveController.getLeftX();

		// Turns the power into a speed
		if(enablePrecisionDrive()){
			power = Math.pow(power, 3);
			speed = power * Drive.POWER_SPEED_RATIO_MPS_PREICISION;	// Use precision mode curve
		} else {
			power = Math.pow(power, 5);
			speed = power * Drive.POWER_SPEED_RATIO_MPS;	// Use regular curve
		}

		// Limits the acceleration when under driver control
		//speed = yLimiter.calculate(speed);

		return speed;
	}

	/**
	 * Gets the rotate speed
	 * <p>Clockwise is positive, converted to chassis speed standards in Drive.teleopDrive()
	 * <p>This measures rotatation around the Z axis
	 * 
	 * @return rotateSpeed
	 */
	public double getRotateSpeed() {
		double speed;
		double power = driveController.getRightX();
		power = Math.pow(power, 5);

		// Turns the power into a speed
		speed = MathUtil.clamp(power, -0.2, 0.2) * Drive.MAX_ROTATE_SPEED; // Use regular curve in radians per second
		
		// Limits the acceleration when under driver control
		//speed = rotateLimiter.calculate(speed);

		return speed;
	}
	
	/**
	 * Pressing A button will zero yaw in case of emergency
	 * 
	 * @return zeroYaw
	 */
	public boolean zeroYaw() {
		//return driveController.getLeftStickButtonPressed();
		return driveController.getDButtonPressed();
	}
	
	/**
	 * F Switch will enable precision control
	 * 
	 * @return precisionControl
	 */
	public boolean enablePrecisionDrive() {
		//return driveController.getRightTriggerAxis() > 0.05;
		return driveController.getFTwoPosSwitch();
	}

	public boolean enableFieldDrive() {
		/*if(driveController.getLeftTriggerAxis() > 0.05) {
			fieldDrive = !fieldDrive;
		}*/
		//return driveController.getLeftTriggerAxis() < 0.05;
		return !driveController.getETwoPosSwitch();
	}

	// Targeting

	/***
	 * Pressing the A button will toggle speaker targeting mode
	 * @return Drive controller A button pressed
	 */
	public boolean toggleSpeakerTargeting() {
		//return driveController.getCButtonUp();
		return false;
	}

	// Stage Functions

	/***
	 * Uses April tags to align the robot with the target & adjust the arm angle for the target distance.
	 * 		The driver will lose control of rotating the robot and adjusting the arm
	 * Pressing the D button will align the robot with the april tag while still allowing the driver to drive (crab drive)
	 * @return Drive controller D button pressed
	 */
	public boolean alignWithAprilTagAndDrive() {
		double value;

		value = driveController.getBThreePosSwitch();
		// switch down(-1) - auto
		if (value == 1) {
			return true;
		} else {
			return false;
		}
	}

	/***
	 * Pressing the DPAD DOWN button will lock the drive wheels
	 * @return Drive controller DPAD DOWN button pressed
	 */
	/*public boolean lockDriveWheels() {
		return driveController.getPOV() == 180;
	}*/

	/**
	 * <p> Checks if the B button is held down on the drive controller
	 * <p> Sets the LEDs to party mode
	 * @return Drive controller B button held down
	 */
	public boolean enablePartyMode() {
		double value;

		value = driveController.getCThreePosSwitch();
		// switch down(-1 on controller axis) - party mode
		if (value == 1) {
			return true;
		} else {
			return false;
		}
	}


	/****************************************************************************************** 
    *
    *    MANIPULATOR FUNCTIONS
    * 
    ******************************************************************************************/
	// Intake & outtake
	/***
	 * Holding the A button will run the intake motors
	 * @return Manipulator controller A button held down
	 */
	public boolean runIntake() {
		return manipulatorController.getLeftTriggerAxis() > 0.05;
	}

	/***
	 * Holding the A button will run the intake motors
	 * @return Manipulator controller Y button held down
	 */
	public boolean overrideIntake() {
		return manipulatorController.getYButton();
	}

	/***
	 * Pressing the left bumper will eject the note from the intake
	 * @return Manipulator controller left bumper pressed
	 */
	public boolean ejectNote() {
		return manipulatorController.getLeftBumper();
	}

	/***
	 * <p> Shoots from multiple distances
	 * <p>		Adjusts arm angle only.  Rotation not controlled.
	 * <p> Pressing the right bumper will shoot the note softly
	 * <p> For AprilTag functions
	 * @return Manipulator controller right bumper pressed
	 */
	/* To be deleted */
	public boolean enableAutoShoot() {
		return manipulatorController.getRightBumper();
	}

	
	// Shooting
	/***
	 * Holding the right trigger will run the shooter motors
	 * @return Manipulator controller right trigger held in
	 */
	public boolean enableShooter() {
		return manipulatorController.getRightTriggerAxis() > 0.05;
	}
	//dump shot
	/*public boolean doDumpShots() {
		return manipulatorController.getBackButtonPressed();
	} */

	/***
	 * Starts the shooter motors only.  Ring not fed into shooter yet.
	 * Pressing the Start button will start the shooting processs
	 * @return Manipulator controller Start button pressed
	 */
	public boolean startShooterWheels() {
		return manipulatorController.getStartButtonPressed();
	}
	
	/***
	 * Pressing the Back button will stop the shooter wheels
	 * @return Manipulator controller Stop button pressed
	 */
	//public boolean stopShooterWheels() {
	//	return manipulatorController.getBackButtonPressed();
	//}

	// Positioning

	/***
	 * Pressing the A button will move the arm to the resting/ground pickup position
	 * @return Manipulator controller A button pressed
	 */
	public boolean moveToRestPosition() {
		return manipulatorController.getXButtonPressed();
	}

	// Arm movement
	/***
	 * Pressing the DPAD UP button will move the arm upwards
	 * @return Manipulator controller DPAD UP button pressed
	 */
	public boolean moveArmUp() {
		int angle;

		angle =  manipulatorController.getPOV();
		if ((angle == 0) || (angle == 45)  || (angle == 315))
		{
		    return true;
		}
		else
		{
			return false;
		}
	}

	/***
	 * Pressing the DPAD DOWN button will move the arm downward
	 * @return Manipulator controller DPAD DOWN button pressed
	 */
	public boolean moveArmDown() {
		int angle;

		angle =  manipulatorController.getPOV();
		if ((angle == 180) || (angle == 225)  || (angle == 135))
		{
		    return true;
		}
		else
		{
			return false;
		}
	}

	/****************************************************************************************** 
    *
    *    MISC FUNCTIONS
    * 
    ******************************************************************************************/
	/**
	 * Checks if the start button is pressed on the drive controller.
	 * 
	 * @return startButtonPressed
	 */
	public boolean autoKill() {
		return false;
	}
}
// End of the Controls class
