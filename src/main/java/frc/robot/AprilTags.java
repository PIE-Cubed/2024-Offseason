package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AprilTags {
    private NetworkTable aprilTagTable;

    private boolean isRed = false;

    private final double MAX_DISTANCE_FT = 6;   // Maximum distance to speaker until its too unreliable to shoot
    private final double CUBED_CONST = 0.0331;      // Old: 0.00000325 w cm
    private final double SQUARED_CONST = 1.2643;    // Old: 0.00317 w cm
    private final double SINGLE_CONST = 17.396;       // Old: 1.09 w cm
    private final double OFFSET = 277.39;              // Old: 228 w cm

    private final int ALL_APRILTAGS_PIPELINE = 0;
    private final int RED_SPEAKER_APRILTAG_PIPELINE = 1;
    private final int BLUE_SPEAKER_APRILTAG_PIPELINE = 2;

    private final double ANGLE_4_1 = 325;
    private final double ANGLE_5 = 330;
    private final double ANGLE_6 = 335;
    private final double ANGLE_7 = 339;
    private final double ANGLE_8 = 342;
    private final double ANGLE_9 = 345;
    private final double ANGLE_10 = 347;
    private final double ANGLE_11 = 349;
    private final double ANGLE_12 = 350.5;
    private final double ANGLE_13 = 351;
    private final double ANGLE_14 = 351.25;

    public AprilTags(boolean isRed) {
        //System.out.println("[INFO] >> Configuring limelight...");
        // Turn off the limelight LEDs so they don't blind people
        LimelightHelpers.setLEDMode_ForceOff("limelight");

        //aprilTagTable = NetworkTableInstance.getDefault().getTable("limelight");
        //System.out.println("[INFO] >> Getting alliance...");
        this.isRed = isRed;

        
    }

    /**
     * <p> Calculate the angle the arm needs to be at to get into the speaker
     * <p> Currently only uses apriltag 7
     * @param odometryDistance Set to -1 to just use AprilTags
     * @return The elevation of the arm to shoot into the speaker
     *         <p> -1 if it fails to get the AprilTag
     */
    public double calculateArmAngleToShoot(double odometryDistance) {
        double delta;
        double angle;


        //double distance = getDistanceToSpeakerFeet() * 30.48;
        //double angle = (CUBED_CONST * Math.pow(distance, 3)) - (SQUARED_CONST * Math.pow(distance, 2)) + (SINGLE_CONST * distance) + OFFSET;
        
        // Use AprilTags when 'odometryDistance' is -1, else use 'odometryDistance'
        double distance = (odometryDistance == -1) ? getDistanceToSpeakerFeet() : odometryDistance;
        if(distance == -1) {
            System.err.println("ERROR: Unable to get AprilTag");
            return -1;
        }

        /* 5.46 feet @ 325 degrees */
        if ((distance >= 4.1)  &&  (distance < 5.0))  {
            delta = distance - 4.1;
            angle = (delta * (ANGLE_5 - ANGLE_4_1)) + ANGLE_4_1;
        }
        /* 5.46 feet @ 325 degrees */
        else if ((distance >= 5)  &&  (distance < 6.0))  {
            delta = distance - 5;
            angle = (delta * (ANGLE_6 - ANGLE_5)) + ANGLE_5;
        }
        /* 6 feet @ 330 degrees */
        else if ((distance >= 6.0)  &&  (distance < 7.0))  {
            delta = distance - 6;
            angle = (delta * (ANGLE_7 - ANGLE_6)) + ANGLE_6;
        }
        /* 7 feet @ 333 degrees */
        else if ((distance >= 7.0)  &&  (distance < 8.0))  {
            delta = distance - 7;
            angle = (delta * (ANGLE_8 - ANGLE_7)) + ANGLE_7;
        }
        /* 8 feet @ 338 degrees */
        else if ((distance >= 8.0)  &&  (distance < 9.0))  {
            delta = distance - 8;
            angle = (delta * (ANGLE_9 - ANGLE_8)) + ANGLE_8;
        }
        /* 9 feet @ 343.5 degrees */
        else if ((distance >= 9.0)  &&  (distance < 10.0))  {
            delta = distance - 9;
            angle = (delta * (ANGLE_10 - ANGLE_9)) + ANGLE_9;
        }
        /* 10 feet @ 345 degrees */
        else if ((distance >= 10.0)  &&  (distance < 11.0))  {
            delta = distance - 10;
            angle = (delta * (ANGLE_11 - ANGLE_10)) + ANGLE_10;
        }
        /* 11 feet @ 347.5 degrees */
        else if ((distance >= 11.0)  &&  (distance < 12.0))  {
            delta = distance - 11;
            angle = (delta * (ANGLE_12 - ANGLE_11)) + ANGLE_11;
        }
        /* 12 feet @ 349.5 degrees */
        else if ((distance >= 12.0)  &&  (distance < 13.0))  {
            delta = distance - 12;
            angle = (delta * (ANGLE_13 - ANGLE_12)) + ANGLE_12;
        }
        /* 12 feet @ 349.5 degrees */
        else if ((distance >= 13.0)  &&  (distance < 14.0))  {
            delta = distance - 13;
            angle = (delta * (ANGLE_14 - ANGLE_13)) + ANGLE_13;
        }
        else  {
            angle = 325;   // rest angle
        }
        /* 13 feet @ 351 degrees */

        return angle;
    }

    /**
     * <p>Gets the distance to the target AprilTag in meters
     * <p>The id param is only used to ensure the correct AprilTag is found
     * <p>Returns -1 if it fails to get the AprilTag
     * <p>Reports a ~3-4in greater distance
     * <p>Must call 'setSpeakerPipeline' first
     * @return Distance to AprilTag in meters
     */
    public double getDistanceToSpeakerFeet() {
        if(validApriltagInView()) {
            /* Pose3d:
             *  x: The horizontal offset
             *  y: The vertical offset
             *  z: The forward offset
             */
            Pose3d targetpose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
            double x = targetpose.getX();
            double z = targetpose.getZ();

            double distance = Math.sqrt((x*x) + (z*z)); // Calculate distance with pythagorean's formula

            return Units.metersToFeet(distance);
        }
        return -1;
    }

    /**
     * Checks if the robot is too far to shoot
     * @return If the robot is too far away from the speaker
     */
    public boolean outOfRange() {
        return Units.metersToFeet(getDistanceToSpeakerFeet()) > MAX_DISTANCE_FT;
    }

    /**
     * Gets the offset of the AprilTag to the center of the limelight's view
     * @return The degree offset
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX("limelight");
    }

    /**
     * Returns if there is a valid target(AprilTag or Color Target) in view
     */
    public boolean validApriltagInView() {
        return LimelightHelpers.getTV("limelight");
    }

    /**
     * <p> Sets the limelight pipeline for the speaker
     * <p> Pipeline 1 for Apriltag 4(Red)
     * <p> Pipeline 2 for Apriltag 7(Blue)
     */
    public void setSpeakerPipeline() {
        if(isRed){
            LimelightHelpers.setPipelineIndex("limelight", RED_SPEAKER_APRILTAG_PIPELINE);  // AprilTag 4
        }
        else {
            LimelightHelpers.setPipelineIndex("limelight", BLUE_SPEAKER_APRILTAG_PIPELINE);  // AprilTag 7
        }
    }

    /**
     * <p> Sets the limelight pipeline for the speaker
     * <p> Use this to manually set the speaker pipeline
     * <p> Pipeline 1 for Apriltag 4(Red)
     * <p> Pipeline 2 for Apriltag 7(Blue)
     * @param isRed If we're on the red alliance
     */
    public void setSpeakerPipeline(boolean isRed) {
        if(isRed){
            LimelightHelpers.setPipelineIndex("limelight", 0);  // AprilTag 4
        }
        else {
            LimelightHelpers.setPipelineIndex("limelight", 1);  // AprilTag 7
        }
    }

    /**
     * <p> Sets the limelight pipeline with no filter
     * <p> Uses pipeline 0
     */
    public void setAllPipeline() {
        LimelightHelpers.setPipelineIndex("limelight", ALL_APRILTAGS_PIPELINE);
    }

    /**
     * Sets the limelight pipeline
     * @param pipeline The pipeline ID
     */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex("limelight", pipeline);
    }

    /****************************************************************************************** 
    *
    *    TEST FUNCTIONS
    * 
    ******************************************************************************************/  
    public void testAprilTagXY() {
        /*
         * tx:
         *  Horizontal offset from crosshair to target
         *  Positive is to the right relative to the camera
         *  LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees
         * ty:
         *  Vertical offset from crosshair to target
         *  Positive is downwards relative to the camera
         *  LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees
         * ta:
         *  Target area percent, 0% - 100% of image
         */
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        double ta = LimelightHelpers.getTA("limelight");

        // Post to smart dashboard
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
    }

    /**
     * <p> Gets the currently targeted AprilTag's field ID
     * @return Targeted AprilTag's field ID
     */
    public int getAprilTagID() {
        return (int)LimelightHelpers.getFiducialID("limelight");
    }

    /**
     * <p> Sets the status of the LEDs
     * @param on
     */
    public void setLED(boolean on) {
        if(on) {
            LimelightHelpers.setLEDMode_ForceOn("limelight");
        }
        else {
            LimelightHelpers.setLEDMode_ForceOff("limelight");
        }
    }

    /**
     * </p>Gets the target's color
     * </p>Make sure to set the limelight to the color pipeline first!
     */
    public void printTargetColor() {
        double hsvColor[] = LimelightHelpers.getTargetColor("limelight");

        System.out.println("Hue: " + hsvColor[0] + " | Saturation: " + hsvColor[1] + " | Value: " + hsvColor[2]);
    }
}
