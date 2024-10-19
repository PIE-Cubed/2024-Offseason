// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.struct.ArmFeedforwardStruct;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // STATUS CODES
	public static final int FAIL = -1;
	public static final int PASS =  1;
	public static final int DONE =  2;
	public static final int CONT =  3;

	// Object creation
  NetworkTableInstance FCSInfo;
	CustomTables         nTables;
  AprilTags            apriltags;
	Odometry             position;
	Controls             controls;
  Shooter              shooter;
  Grabber              grabber;
	Drive                drive;
	Auto                 auto;
  Arm                  arm;
  LED                  led;

	// Variables
	private int status = CONT;
	private boolean firstTime = true;
  //private boolean shooterState = false;
 // private boolean autoShooterState = false;

	// Auto path
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private String m_autoSelected;
  
  // Statuses for each "module"   
  private int armIntakeStatus   = CONT;
  private int shooterStatus     = CONT;
  private int grabberStatus     = CONT;
  private int armRestStatus     = CONT;
  private int moveStatus        = CONT;
  private int armStatus         = CONT;
  private int armRotateStatus   = CONT;
  private int restingStatus     = CONT;
  
  // Statuses for AprilTag targeting
  private int apriltagArmStatus     = CONT;
  private int apriltagAlignedStatus = CONT;
  
  private boolean shooterSpinning;

  private double startTime = 0;
  private int iterCount = 0;

  /* arm states */
  enum ArmState { TELEOP, REST, SHOOT, AUTO_SHOOT };
  private ArmState armState = ArmState.TELEOP;

  /* TeleOp States */
  enum TeleopState { TELEOP, TARGET, CRAB_SHOOT };
  private TeleopState teleopState = TeleopState.TELEOP;

  /* Shooter States */
  enum ShooterState { TELEOP, SPEAKER_SHOOT, AUTO_SHOOT, AUTO_AIM_ROTATE, AUTO_AIM_ROTATE_SHOOT };
  private ShooterState shooterState = ShooterState.TELEOP;

  /* Grabber States */
  enum GrabberState { TELEOP, OVERRIDE, SHOOTER_CONTROL };
  private GrabberState grabberState = GrabberState.TELEOP;


	/**
	 * Constructor
	 */
	public Robot() {
    //System.out.println("[INFO] >> Initializing chooser(s)...");
    
    m_chooser = new SendableChooser<>();

		// Instance getters
    //System.out.println("[INFO] >> Initializing instance getters...");
        
		nTables  = CustomTables.getInstance();
    FCSInfo = NetworkTableInstance.getDefault();

		// Instance creation
    grabber   = Grabber.getInstance();
    apriltags = new AprilTags(nTables.getIsRedAlliance());
		drive     = new Drive(apriltags);
		controls  = new Controls();
		position  = new Odometry(drive);
    shooter   = new Shooter();
    arm       = new Arm();
		auto      = new Auto(drive, position, arm, grabber, shooter, apriltags);
    led       = new LED();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //System.out.println("[INFO] >> Robot init running.");

    // Start the camera server
    //System.out.println("[INFO] >> Starting camera server...");
    //CameraServer.startAutomaticCapture();
    
    // Auto selection
    //System.out.println("[INFO] >> Configuring auto...");
    m_chooser.setDefaultOption("Speaker Center Auto", "Speaker Center Auto");
    m_chooser.addOption("Amp Side Auto", "Amp Side Auto");
    m_chooser.addOption("Feed Side Auto", "Feed Side Auto");
    m_chooser.addOption("No Auto", "No Auto");
    m_chooser.addOption("Speaker Center Just Shoot Auto", "Speaker Center Just Shoot Auto");
    m_chooser.addOption("Speaker Center Get Distance Points Auto", "Speaker Center Get Distance Points Auto");
    SmartDashboard.putData("Auto Selector", m_chooser);

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    armRotateStatus = Robot.CONT;
    status = Robot.CONT;

    shooterSpinning = false;
    apriltags.setLED(false);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //position.updatePoseTrackers();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    System.out.println("[INFO] >> Auto mode selected.");
    m_autoSelected = m_chooser.getSelected();

    System.out.println("[INFO] >> Getting alliance color...");
        
    auto.isRed = this.nTables.getIsRedAlliance();
    
    System.out.println("[INFO] >> Getting alliance color...");
        
    if(!auto.isRed) {
        auto.allianceAngleModifier = -1;
    }
    
    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    armRotateStatus = Robot.CONT;
    status = Robot.CONT;

    System.out.println("[INFO] >> Auto program selected: " + m_autoSelected);    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (status == Robot.CONT) {
      switch (m_autoSelected) {
        case "Speaker Center Auto":
          status = auto.speakerPositionCenter();
          break;

        case "Amp Side Auto":
          status = auto.speakerPositionAmp();
          break;

        case "Feed Side Auto":
          status = auto.speakerPositionFeed();
          break;

        case "Speaker Center Just Shoot Auto":
          status = auto.speakerShootWithoutMoving();
          break;

        case "Speaker Center Get Distance Points Auto":
          status = auto.speakerShootMoveOut();
          break;

        case "No Auto":
          status = Robot.DONE;
          break;
          
        default:
          System.out.println("[INFO] >> Running default auto.");
          //status = auto.speakerPositionCenter();
          break;
      }

      //status = auto.speakerPositionCenter();
    } 
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //System.out.println("[INFO] >> TeleOp mode selected.");

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    armRotateStatus = Robot.CONT;
    status = Robot.CONT;

    shooterSpinning = false;
    //apriltags.setSpeakerPipeline();
    //System.out.println(apriltags.getDistanceToSpeakerFeet());

    // Turn on the shooter motors
    //shooter.spinup();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Allows for driving the robot
    wheelControl();

    // Allows for controlling the arm
    armControl();

    // Allows for controlling the grabber
    grabberControl();
    
    // Allows for shooting notes
    shooterControl();
    
    // Allows for controlling the LEDs
    ledControl();
    
    // Drivers check this to see if they grabbed a note
    SmartDashboard.putBoolean("Grabber has Note", grabber.noteDetected());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println("[INFO] >> Test mode selected.");

    // Initialize Shuffleboard
    SmartDashboard.putNumber("Rotation Power", 0.0);

    // Reset the robot statuses. This ensures that we don't need to restart the code after every time we
    // run the robot.
    grabberStatus = Robot.CONT;
    armStatus = Robot.CONT;
    armRotateStatus = Robot.CONT;
    status = Robot.CONT;

    //driveDistance = false;
    startTime = System.currentTimeMillis();
    iterCount = 0;

    testStatus = Robot.CONT;
    on = false;
    SmartDashboard.putNumber("Shoot Angle", 327);
  }
  
  int testStatus = Robot.CONT;
  boolean on = false;

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Read values from shuffleboard
    //double shooterPower = SmartDashboard.getNumber("Shooter Power", 0.0);
    //double grabberPower = SmartDashboard.getNumber("Grabber Power", 0.0);
    //double rotateAngle = SmartDashboard.getNumber("Rotation Angle", 0.0);
    //double power = SmartDashboard.getNumber("Rotation Power", 0.0);

    //apriltags.setSpeakerPipeline();
    //System.out.println(apriltags.calculateArmAngleToShoot());

    //apriltags.setSpeakerPipeline();
    //if(drive.alignWithAprilTag() == DONE) {
    //  System.out.println("Finished in: " + (System.currentTimeMillis() - startTime) + "ms | " + iterCount + " iterations");
    //  startTime = System.currentTimeMillis();
    //  iterCount = 0;
    //}
    //else {
    //  iterCount++;
    //}

    /*if (armStatus == Robot.CONT) {
      armStatus = arm.extendArmToPosition(arm.ARM_INTAKE_POSITION, 0.3); //145
    }

    System.out.println(arm.getExtendPosition());*/

    //System.out.println(drive.getFLRot());
    //System.out.println(Math.toDegrees(drive.getBRAngle()));
    //testTeleopDrive();
  
    // Test shooter
    //shooter.startShooting(shooterPower);

    // Test grabber
    // Does not automatically stop the grabber or check for a note
    //grabber.setMotorPower(1);
    // Automatically stops the grabber when a note is detected
    //if (grabberStatus == Robot.CONT) {
    //  grabberStatus = grabber.intakeOutake(true, false);
    //}

    // Retrieve RGB, IR, and proximity values from the color sensor
    //grabber.testColorSensor();
    //SmartDashboard.putBoolean("Note Detected", grabber.noteDetected());

    // Test AprilTags
    //drive.alignWithAprilTag();
    /*arm.maintainPosition(SmartDashboard.getNumber("Arm Angle", arm.ARM_REST_POSITION_DEGREES));
    if (controls.enableShooter()) {
      status = auto.autoDelay(1);
      shooter.spinup();

      if (status == DONE)
        grabber.setMotorPower(grabber.INTAKE_POWER);
    }
    else {
      status = Robot.CONT;
      shooter.spindown();
      grabber.setMotorPower(0);
    }*/
    //System.out.println(apriltags.getDistanceToSpeakerFeet());
    

    // Test the auto selection
    //System.out.println("Selected auto: " + m_chooser.getSelected());

    // Test LEDs
    //ledControl();

   // System.out.println("Arm Angle: " + arm.getElevationPosition());

    //arm.extendToRest();
    //arm.rotateToRest(1.25);
    //System.out.println("At Rest Pos: " + arm.getRestButton() + " | " + arm.getElevationPosition());

    // Move the arm to a certain degree
    /*if (armStatus == Robot.CONT) {
      armStatus = arm.rotateArm(355);
    }
    else {
      armStatus = arm.maintainPosition(355);
    }*/

    // Test driving at an angle
    /* 
    if(status == Robot.CONT) {
      status = drive.testAngleDrive(0, 3, 0.3);
    }
    */
    
    // Test arm elevation
    //arm.testElevate();
    //System.out.println("Elevation Encoder", Math.toDegrees(arm.getElevationPosition()));

    // Test driving at an angle
    /*
    if (status == Robot.CONT) {
      status = drive.driveDistanceWithAngle(0, -2, 0.3);
    }*/

    //if(controls.toggleLimelightLED()) {
    //  on = !on;
    //}
    //apriltags.setLED(on);

    // Get the arm extension position
    //System.out.println("Arm position: " + arm.getElevationPosition());
    //System.out.println("Arm extension position: " + arm.getExtendPosition());
    //System.out.println(
    //                   "Intake Button 1 State: " + arm.getIntakeButton1() +
    //                   " | Intake Button 2 State: " + arm.getIntakeButton2() +
    //                   " | Rest Button State: " + arm.getRestButton()
    //);

   // System.out.println("Proximity: " + grabber.getProximity());

    // Get drive controller values
    //System.out.println("Forward speed: " + controls.getForwardSpeed() + " Strafe speed: " + controls.getStrafeSpeed() + " Rotate speed: " + controls.getRotateSpeed());

    System.out.println("Feet: " + apriltags.getDistanceToSpeakerFeet());
    double angle = SmartDashboard.getNumber("Shoot Angle", 327);
    boolean shoot = controls.enableShooter();
    
    if(shoot){
      on = true;
    }

    if(testStatus != Robot.DONE && on) {
      testStatus = auto.testShoot(angle);
    } else {
      shooter.spindown();
      on = false;
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
	 * Controls the wheels in TeleOp
	 */
	private void wheelControl() {
		// Gets the Drive Values
		double  rotateSpeed        = controls.getRotateSpeed();
		double  strafeSpeed        = controls.getStrafeSpeed();
		double  forwardSpeed       = controls.getForwardSpeed();
    boolean driveWhileAligning = controls.alignWithAprilTagAndDrive();
    
		// Gets Manipulator values
		boolean zeroYaw           = controls.zeroYaw();
    boolean fieldDrive        = controls.enableFieldDrive();
    boolean targetSpeaker     = controls.toggleSpeakerTargeting();

    // Zeros the gyro
		if (zeroYaw == true) {
			drive.resetYaw();
		}

    // Targets the speaker
    /*if(targetSpeaker) {
      teleopState = TeleopState.TARGET;
      apriltags.setSpeakerPipeline();   // Set the pipeline depending on alliance color(0,ID4,Red, 1,ID7,Blue)
    }*/
    
    if(teleopState == TeleopState.TELEOP) {
      drive.teleopDrive(forwardSpeed, strafeSpeed, rotateSpeed, fieldDrive);

      if (targetSpeaker == true) {
        teleopState = TeleopState.TARGET;
      }
      else if (driveWhileAligning == true) {
        teleopState = TeleopState.CRAB_SHOOT;
      }
      else {
        teleopState = TeleopState.TELEOP;
      }
    }
    else if(teleopState == TeleopState.TARGET) {
      apriltags.setSpeakerPipeline();   // Set the pipeline depending on alliance color(0,ID4,Red, 1,ID7,Blue)
      int targetStatus = auto.targetSpeaker();
      
      if(targetStatus == DONE) {      // Done rotating, drive forward
        teleopState = TeleopState.TELEOP;
      }
      else if(targetStatus == CONT) { // Not done rotating
        teleopState = TeleopState.TARGET;
      }
      else if(targetStatus == FAIL) { // Can't find AprilTag
        teleopState = TeleopState.TELEOP;
      }
    }
    else if (teleopState == TeleopState.CRAB_SHOOT) {
      drive.maintainShootingWithCrabDrive(forwardSpeed, strafeSpeed);

      if (driveWhileAligning == false) {
        teleopState = TeleopState.TELEOP;
      }
      else {
        teleopState = TeleopState.CRAB_SHOOT;
      }
    }

		// Calculated line of best fit for relationship between rotate speed and drift angle
    // Think its used to travel straight when rotating
    // Will allegedly revisit later to adjust to new motors 
		/**
     *  double angleTransform = ROTATE_SPEED_OFFSET * rotateSpeed;
     * Translation2d velocity = new Translation2d(forwardSpeed, strafeSpeed);
     * Translation2d newVelocity = velocity.rotateBy(new Rotation2d(angleTransform));
     * double newXVel = newVelocity.getX();
     * double newYVel = newVelocity.getY();
     * 
     * drive.teleopDrive(newXVel, newYVel, rotateSpeed, true);
     */
	}

  /**
	 * Controls the LEDs
	 */
  private void ledControl() {
    boolean hasNote       = grabber.noteDetected();
    boolean runningIntake = controls.runIntake();
    boolean partyMode     = controls.enablePartyMode();
    boolean autoMode      = controls.alignWithAprilTagAndDrive();
    
    if(partyMode) {             // Top priority
      led.partyColor();           // Sets the color to rainbow
    } 
    else if (autoMode) {
      led.autoAim();
    }
    else if(hasNote) {        // If the grabber has a note, second priority
      led.capturedNoteColor();    // Sets the color to green
      //drive.rumbleController();
    } 
    else if(runningIntake) {  // If the grabber is running(no note), third priority
      led.gettingNoteColor();     // Sets the color to orange
    } 
    else {                    // Default state
      led.robolionsColor();       // Sets the color to blue-gold
    }
    
    led.updateLED();  // Update LEDs

  }
  
  /**
	 * Controls the arm in TeleOp
	 */
  private void armControl() {
      if(armState == ArmState.TELEOP) {
          double armPos = arm.getElevationPosition();
          
          // Move the arm up/down incrementally
          // Arm must be less than 180 (so it doesn't go into the robot)
          // Or greater than 320 (so it can rise up from rest)
          if(controls.moveArmUp() && (armPos < 180 || armPos > 320)) {
              arm.testElevate(-0.5);
          }
          // Arm must be greater than 0, but not higher than 180 (inside the robot, for positions like rest)
          else if(controls.moveArmDown() && (armPos > 0 && armPos < 180)) {
          arm.testElevate(0.5);
          }
          else {
            arm.testElevate(0);
          }

          // Attempted to shoot and goto rest state
          // Should we exit (stay in teleop), or goto rest? - Jack K
          if(controls.moveToRestPosition() && (controls.enableShooter() || controls.alignWithAprilTagAndDrive())) {
            System.out.println("ARM ERROR: Attempted to shoot or crab shoot while in rest state");
            
            return;
          }

          // Check states
          if(controls.moveToRestPosition())  {
              armState = ArmState.REST;
          }
          else if(controls.enableShooter())  {
              armState = ArmState.SHOOT;
          }
          else if (controls.alignWithAprilTagAndDrive()) {
            armState = ArmState.AUTO_SHOOT;
          }

      }
      // TODO See if the delay is necessary
      else if(armState == ArmState.REST) {
          armStatus = arm.rotateToRest(1.5);
          armRestStatus = auto.autoDelayMS(1500);

          if (armStatus == Robot.DONE || armRestStatus == Robot.DONE) {
              armState = ArmState.TELEOP;
              armRestStatus = Robot.CONT;
              armIntakeStatus = Robot.CONT;
          }
          else {
            armState = ArmState.REST;
          }
      }
      else if(armState == ArmState.SHOOT) {
          // Hands over control of the arm to shooterControl
          if(controls.enableShooter() == false) {
            armState = ArmState.TELEOP;
          }
          else {
            armState = ArmState.SHOOT;
          }
      }
      else if(armState == ArmState.AUTO_SHOOT) {
          arm.maintainPosition(apriltags.calculateArmAngleToShoot());

          // Determine next state
          if(controls.alignWithAprilTagAndDrive() == false) {
            armState = ArmState.TELEOP;
          } 
          else {
            armState = ArmState.AUTO_SHOOT;
          }
      }
  }

  /**
	 * Controls the shooter in TeleOp
	 */
	private void shooterControl() {
    boolean enableShooter     = controls.enableShooter();               // Shoot from against the speaker
    boolean crabShoot         = controls.alignWithAprilTagAndDrive();   // Shoot from multiple distances with auto rotation on target
    boolean startStopShooter  = controls.startShooterWheels();
    int     status;

    if (shooterState == ShooterState.TELEOP)  {
      // Start or stop the shooter wheels, the start button flips the current state
      if(startStopShooter == true) {
        shooterSpinning = !shooterSpinning;
        if(shooterSpinning) {
          shooter.spinup();
        }
        else {
          shooter.spindown();
        }
      }

      // Checks states
      if(enableShooter) {
        shooterState = ShooterState.SPEAKER_SHOOT;
      }
      // Crab shoot
      else if(crabShoot) {
        shooterState = ShooterState.AUTO_AIM_ROTATE;
      }
    }
    // shoot against the speaker.  Will complete when releasing button.
    //  Shooter will spin down
    else if (shooterState == ShooterState.SPEAKER_SHOOT)  {
       status = auto.teleopShoot(enableShooter);

       if (status == Robot.DONE)  {
         shooterState = ShooterState.TELEOP;
       }
       else {
         shooterState = ShooterState.SPEAKER_SHOOT;
       }
    }
    // Shoot from any distance using april tags for arm rotation and
    //  april tags for rotation lock on target
    //  Ring is shot when the manipulator pulls the trigger
    // Basically is the base point for crab drive
    else if (shooterState == ShooterState.AUTO_AIM_ROTATE)  {
        // armControl will set arm angle
        // driveControl will set robot orientation
        // shoot when manipulator pulls trigger
        if (crabShoot == false)  {
            shooterState = ShooterState.TELEOP;
        }
        else if (enableShooter == true)  {
            shooterState = ShooterState.AUTO_AIM_ROTATE_SHOOT;
        }
        else  {
            shooterState = ShooterState.AUTO_AIM_ROTATE;
        }
    }
    else if (shooterState == ShooterState.AUTO_AIM_ROTATE_SHOOT)  {
        status = auto.teleopShootCrabDrive(enableShooter);

        if (status == Robot.DONE)  {
            shooterState = ShooterState.AUTO_AIM_ROTATE;
        }
        else  {
            shooterState = ShooterState.AUTO_AIM_ROTATE_SHOOT;
        }
    }
	}

  /**
	 * Controls the grabber in TeleOp
	 */
  private void grabberControl() {
    boolean enableShooter     = controls.enableShooter();
    boolean enableOverride    = controls.overrideIntake();

    if(grabberState == GrabberState.TELEOP) {
      // Check if we're still in this state, override takes priority
      if(enableOverride) {
        grabberState = GrabberState.OVERRIDE;
      }
      else if(enableShooter) {
        grabberState = GrabberState.SHOOTER_CONTROL;
      }

      grabber.intakeOutake(controls.runIntake(), controls.ejectNote(), false);
    }
    else if(grabberState == GrabberState.OVERRIDE) {
      if(enableOverride == false) {
        grabberState = GrabberState.TELEOP;
      }

      grabber.setMotorPower(grabber.INTAKE_POWER);
    }
    else if(grabberState == GrabberState.SHOOTER_CONTROL) {
      if(enableShooter == false) {
        grabberState = GrabberState.TELEOP;
      }
      else if(enableOverride) {
        grabberState = GrabberState.OVERRIDE;
      }
    }
  }
}