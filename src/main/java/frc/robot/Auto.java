// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Auto {
    // State tracking variables - each variable can only be used in one function at any time
    // All top level routines use firstTime and step, all helper routines have their own variables
    private int step;
    private boolean firstTime = true;
    private boolean teleopShootFirstTime = true;
    private boolean autoShootFirstTime = true;
    public boolean isRed = false;
    public int allianceAngleModifier = 1;

    private long delayEnd = 0; // Stores when delay() should return Robot.DONE
    private boolean delayFirstTime = true;

    
    private int intakeStatus = Robot.CONT;
    private int driveStatus = Robot.CONT;
    private int status = Robot.CONT;
    
    private final double SHOOT1_ANGLE = 325;   // Up against speaker
    
    private double apriltagShootAngle = SHOOT1_ANGLE;   // Start at against the speaker

    // Auto program selection
    //public String selectedAuto = "Speaker Center";

    // Object Creation
    private Drive          drive;
    private Odometry       position;
    private CustomTables   nTables;
    private Arm            arm;
    private Grabber        grabber;
    private Shooter        shooter;
    private AprilTags      apriltags;

    // Constructor
    public Auto(Drive drive, Odometry position, Arm arm, Grabber grabber, Shooter shooter, AprilTags apriltags) {
        this.drive     = drive;
        this.grabber   = grabber;
        this.position = position;
        this.arm       = arm;
        this.nTables   = CustomTables.getInstance();
        this.shooter   = shooter;
        this.apriltags = apriltags;

        this.isRed = nTables.getIsRedAlliance();

        allianceAngleModifier = (isRed) ? 1 : -1;   // Don't modify if blue
    }

    /****************************************************************************************** 
     * 
     *    MANIPULATOR FUNCTIONS
     * 
     ******************************************************************************************/

    /**
     * <p> Shoots into the speaker
     * <p> Moves back to get a note, drives back, then shoots against the speaker
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionCenterNoAprilTags() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            step = 1;
        }

        switch(step) {            
            // Rotate the drive motors to zero
            case 1:
                status = drive.rotateWheelsToAngle(0);
                break;

            // Retract the arm fully to prevent out of bounds issues
            case 2:
                //status = arm.extendArm(23, 0.25);
                status = Robot.DONE;
                break;
            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 3:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Extend the arm to its original position
            case 4:
                //status = arm.extendArm(-16, -0.25);
                status = Robot.DONE;
                arm.maintainPosition(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 5:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.6 second(s)
            case 6:
                status = autoDelayMS(600);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Drive backwards 4 feet
            case 7:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false, true);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4.5, 0.2);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;
            
            // Drive back to the speaker
            case 8:
                arm.maintainPosition(SHOOT1_ANGLE);
                status = drive.driveDistanceWithAngle(0, -5.3, 0.5);
                break;

            // Rotate the arm so it's in the shooting position
            case 9:
                status = arm.rotateArm(SHOOT1_ANGLE);    
                break;

            // Shoot the note
            case 10:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = autoDelay(1);
                break;
            
            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
                grabber.intakeOutake(false, false, true);
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Shoots into the speaker
     * <p> Moves back to get a note and shoot with AprilTags
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionCenter() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            status = Robot.CONT;
            step = 1;

            apriltags.setSpeakerPipeline(isRed);
            System.out.println("Is red: " + isRed);
        }

        switch(step) {            
            // Rotate the drive motors to zero
            case 1:
                status = drive.rotateWheelsToAngle(0);
                break;
            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 2:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            case 3:
                status = autoDelay(1);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;
                        
            // Shoot the note by running the grabber
            case 4:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.75 second(s)
            case 5:
                status = autoDelayMS(750);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Drive backwards 4 1/2 feet
            case 6:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false, true);
                }
                else {
                    grabber.setMotorPower(0);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4.5, 0.2);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            case 7:
                apriltagShootAngle = apriltags.calculateArmAngleToShoot();
                if(apriltagShootAngle == -1){
                    apriltagShootAngle = 341.5; // Just in case, go to 341.5
                    System.out.println("Can't see valid AprilTag");
                } else {
                    System.out.println("Saw a tag but ignoring it and using a fixed position");
                }
               // apriltagShootAngle = 341.5;
                System.out.println("Shooting at " + apriltagShootAngle + "deg");
                
                status = arm.rotateArm(apriltagShootAngle);
                break;

            case 8:
                arm.maintainPosition(apriltagShootAngle);
                status = autoDelayMS(1000);
                break;
            
            // Start the grabber and keep the arm in shooting position
            case 9:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(apriltagShootAngle);
                status = autoDelayMS(500);
                break;
            
            // Finished routine, reset variables, stop motors, and return done
            default:
                System.out.println("Done, status: " + status);
                shooter.stopShooting();
                grabber.intakeOutake(false, false, true);
                arm.testElevate(0);
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Shoots into the speaker
     * <p> Drives back, rotates, drives and gets note, then shoots with AprilTags
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionAmp() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            apriltags.setSpeakerPipeline(isRed);

            System.out.println("Is Red: " + isRed);
            
            step = 1;
        }

        switch(step) {   
            // Rotate the drive motors to zero
            case 1:
                shooter.spinup();
                status = drive.rotateWheelsToAngle(0);
                break;
            
            case 2:
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;
    
            // Shoot the note by running the grabber
            case 3:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 0.75 second(s)
            case 4:
                status = autoDelayMS(750);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Rotate the arm to its resting position, and turn off the shooter & grabber
            case 5:            
                grabber.intakeOutake(false, false, true);
                status = Robot.DONE;       
                break;

            case 6:
                status = drive.driveDistanceWithAngle(0, 1.4, 0.5);
                break;

            case 7:
                status = drive.rotateRobot(Math.toRadians(allianceAngleModifier * 50));
                break;
                
            // Rotate the wheels back to zero before driving forward
            case 8:
                driveStatus = drive.rotateWheelsToAngle(0);            

                if(driveStatus == Robot.DONE) {
                    driveStatus = Robot.CONT;
                    status = Robot.DONE;
                } 
                else {
                    status = Robot.CONT;
                }
                break;

            // Drive forwards 4.5 feet and pick up a note
            case 9:
                if (intakeStatus == Robot.CONT) {
                    intakeStatus = grabber.intakeOutake(true, false, true);
                }
                
                if (driveStatus == Robot.CONT) {
                    driveStatus = drive.driveDistanceWithAngle(0, 4.5, 0.5);
                }
                
                if (intakeStatus == Robot.DONE && driveStatus == Robot.DONE) {
                    driveStatus = Robot.CONT;
                    intakeStatus = Robot.CONT;
                    status = Robot.DONE;
                }
                else {
                    status = Robot.CONT;
                }
            
                break;

            // Align with the speaker to shoot
            case 10:
                status = drive.alignWithAprilTag();
                break;

            // Rotate to rest
            case 11:
                status = arm.rotateToRest(1.5);
                break;

            // Raise arm to shooting angle
            case 12:
                //April tag distance is 9.49 FT, low by 1-2 degrees
                apriltagShootAngle = apriltags.calculateArmAngleToShoot();

                if(apriltagShootAngle == -1) {
                    apriltagShootAngle = 343;
                    System.out.println("Cannot see AprilTag, shooting at 331.5");
                }

                status = arm.rotateArm(apriltagShootAngle);
                break;

            case 13:
                arm.maintainPosition(apriltagShootAngle);
                status = autoDelayMS(1000);
                break;
            
            // Start the grabber and keep the arm in shooting position
            case 14:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(apriltagShootAngle);
                status = autoDelayMS(500);
                break;

            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
                grabber.intakeOutake(false, false, true);
                arm.testElevate(0);
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Shoots, then backs up to clear note and stage, drives out, then back
     * @return Robot status, CONT or DONE
     */
    public int speakerPositionFeed() {
        if(firstTime == true) {
            firstTime = false;
            intakeStatus = Robot.CONT;
            driveStatus = Robot.CONT;
            apriltags.setSpeakerPipeline(isRed);

            System.out.println("Is Red: " + isRed);

            step = 1;
        }

        switch(step) {
            // Rotate the drive motors to zero
            case 1:
            shooter.spinup();
                status = drive.rotateWheelsToAngle(0);
                break;
            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 2:
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            // Assume the robot shot the note after 0.75 second(s)
            case 3:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = autoDelayMS(750);
                break;

            // Turn off grabber and drive
            case 6:            
                grabber.intakeOutake(false, false, true);
                status = drive.driveDistanceWithAngle(0, 6, 0.5);
                break;

            // Rotate the robot 57 degrees
            case 7:
                status = drive.rotateRobot(Math.toRadians(allianceAngleModifier * -50));
                break;

            // Rotate the wheels back to zero before driving forward
            case 8:
                status = drive.rotateWheelsToAngle(0);            
                break;
            
            // Drive forwards to get out of alliance area
            case 9:
                status = drive.driveDistanceWithAngle(0, 3, 0.5);
                break;

            // Drive backwards to get back into alliance area
            case 10:
                status = drive.driveDistanceWithAngle(0, -3.5, 0.5);
                break;

            // Finished routine, reset variables, stop motors, and return done
            default:
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Faces the speaker
     * <p> Fail stops the auto immediatly, resets, and returns fail
     * @return Robot status, CONT, DONE, or FAIL
     */
    public int targetSpeaker() {
        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {
            case 1:
                status = drive.alignWithAprilTag();
                break;
            case 2:
                status = drive.rotateWheelsToAngle(0);
                break;
            default:
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }
        else if(status == Robot.FAIL) { // Fail, reset and return fail
            System.out.println("No Valid AprilTag in Frame!!!");
            step = 1;
            firstTime = true;
            return Robot.FAIL;
        }

        return Robot.CONT;
    }

    /**
     * <p> Shoot into the speaker and stop
     * @return Robot status, CONT or DONE
     */
    public int speakerShootWithoutMoving() {
        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 1:
                shooter.spinup();
                status = autoDelayMS(1000);
                break;
                        
            // Shoot the note by running the grabber
            case 2:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 1 second(s)
            case 3:
                status = autoDelay(1);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
                grabber.intakeOutake(false, false, false);
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Shoot into the speaker
     * <p> Drive out and back into alliance safe zone for points
     * @return Robot status, CONT or DONE
     */
    public int speakerShootMoveOut() {
        if(firstTime == true) {
            firstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -26 (333) degrees from 54
            case 1:
                shooter.spinup();
                status = autoDelayMS(1000);
                break;
                        
            // Shoot the note by running the grabber
            case 2:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                status = Robot.DONE;
                break;

            // Assume the robot shot the note after 1 second(s)
            case 3:
                status = autoDelay(1);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;

            // Drive out 4 feet to get extra points
            case 4:
                status = drive.driveDistanceWithAngle(0, 4, 0.5);
                break;

            // Wait a bit to be sure
            case 5:
                status = autoDelayMS(500);
                break;

            // Drive back 4 feet to get extra points
            case 6:
                status = drive.driveDistanceWithAngle(0, -4, 0.5);
                break;

            // Finished routine, reset variables, stop motors, and return done
            default:
                shooter.stopShooting();
                grabber.intakeOutake(false, false, false);
                step = 1;
                firstTime = true;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Shoot a note from against the speaker
     * <p> Release right trigger to stop shooting
     * @param shooterEnable Whether to continue shooting or not
     * @return Robot status, CONT or DONE
     */
    public int teleopShoot(boolean shooterEnable) {
        int status = Robot.CONT;

        if(teleopShootFirstTime == true) {
            teleopShootFirstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -23 (336) degrees from 54
            case 1:
                shooter.spinup();
                status = arm.rotateArm(SHOOT1_ANGLE);
                break;

            case 2:
                status = autoDelayMS(750);
                arm.maintainPosition(SHOOT1_ANGLE);
                break;
            
            // Start the grabber and keep the arm in shooting position
            case 3:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(SHOOT1_ANGLE);
                
                if(shooterEnable) {
                    status = Robot.CONT;
                }
                else {
                    status = Robot.DONE;
                }
                break;
            
            default:
                teleopShootFirstTime = true;
                grabber.intakeOutake(false, false, false);
                arm.disableRotation();
                shooter.spindown();
                step = 1;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Rotates the robot and elevates the arm so that they face the speaker
     * <p> Allows the driver to continue to drive
     * @return Robot status, CONT or DONE
     */
    public int teleopShootCrabDrive(boolean shooterEnable) {
        //int intakeStatus = Robot.CONT;
        //int driveStatus = Robot.CONT;
        int status = Robot.CONT;

        if(teleopShootFirstTime == true) {
            teleopShootFirstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -23 (336) degrees from 54
            case 1:
                shooter.spinup();
                //status = autoDelayMS(500);
                status = Robot.DONE;
                break;
            
            // Start the grabber and keep the arm in shooting position
            case 2:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                
                if(shooterEnable) {
                    status = Robot.CONT;
                }
                else {
                    status = Robot.DONE;
                }
                break;
            
            default:
                teleopShootFirstTime = true;
                grabber.intakeOutake(false, false, false);
                arm.disableRotation();
                step = 1;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /**
     * <p> Automatically elevates the arm so the shooter is aiming at the speaker
     * @return Robot status, CONT or DONE
     */
    public int apriltagShoot(boolean shooterEnable) {
        int status = Robot.CONT;

        if(autoShootFirstTime == true) {
            apriltags.setSpeakerPipeline(isRed);
            autoShootFirstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -23 (336) degrees from 54
            case 1:
                shooter.spinup();
                apriltagShootAngle = apriltags.calculateArmAngleToShoot();
                //System.out.println(apriltagShootAngle);
                status = arm.rotateArm(apriltagShootAngle);
                break;

            case 2:
                arm.maintainPosition(apriltagShootAngle);
                status = autoDelayMS(1000);
                break;
            
            // Start the grabber and keep the arm in shooting position
            case 3:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(apriltagShootAngle);
                
                //if(shooterEnable) {
                //    status = Robot.CONT;
                //}
                //else {
                status = autoDelayMS(500);
                //status = Robot.DONE;
                //}
                break;
            
            case 4:
                status = arm.rotateToRest(1.5);
                break;

            default:
                autoShootFirstTime = true;
                grabber.intakeOutake(false, false, false);
                arm.disableRotation();
                shooter.spindown();
                step = 1;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }

    /*
     * Set Teleop shooting first time to true
     */
    public void resetTeleopShoot() {
        teleopShootFirstTime = true;
    }

    /*
     * Set auto shooting first time to true
     */
    public void resetAutoShoot() {
        autoShootFirstTime = true;
    }

    /****************************************************************************************** 
     *
     *    HELPER FUNCTIONS
     * 
     ******************************************************************************************/
    /**
     * Delays the program for a set number of seconds.
     * 
     * @param seconds
     * @return status
     */
    public int autoDelay(long seconds) {
        long currentMS = System.currentTimeMillis();

        if (delayFirstTime) {
            delayEnd = currentMS + (seconds * 1000);
            delayFirstTime = false;
        }

        if (currentMS > delayEnd) {
            delayFirstTime = true;
            return Robot.DONE;
        }
        return Robot.CONT;
    }

    /**
     * Delays the program for a set number of milliseconds.
     * 
     * @param seconds
     * @return status
     */
    public int autoDelayMS(long ms) {
        long currentMS = System.currentTimeMillis();

        if (delayFirstTime) {
            delayEnd = currentMS + ms;
            delayFirstTime = false;
        }

        if (currentMS > delayEnd) {
            delayFirstTime = true;
            return Robot.DONE;
        }
        return Robot.CONT;
    }


    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/    
    /**
     * <p> Shoots a note at the given angle
     * @return Robot status, CONT or DONE
     */
    public int testShoot(double angle) {
        int status = Robot.CONT;

        if(teleopShootFirstTime == true) {
            teleopShootFirstTime = false;
            step = 1;
        }

        switch(step) {            
            // Start the shooter motors and rotate the arm to -23 (336) degrees from 54
            case 1:
                shooter.spinup();
                status = arm.rotateArm(angle);
                break;

            case 2:
                status = autoDelayMS(750);
                arm.maintainPosition(angle);
                break;
            
            // Start the grabber and keep the arm in shooting position
            case 3:
                grabber.setMotorPower(grabber.INTAKE_POWER);
                arm.maintainPosition(angle);
                
                status = autoDelayMS(500);
                break;
            
            default:
                teleopShootFirstTime = true;
                grabber.intakeOutake(false, false, false);
                arm.disableRotation();
                shooter.spindown();
                step = 1;
                return Robot.DONE;
        }

        // Done current step, goto next one
        if(status == Robot.DONE) {
            step++;
        }

        return Robot.CONT;
    }


}
// End of the Auto class
