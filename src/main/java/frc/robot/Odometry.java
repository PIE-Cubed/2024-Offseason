package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * </p> Odometry/Pose estimation class
 * 
 * </p> Still has encoder-only estimation just in case
 */
public class Odometry {
    private final int REQUIRED_APRILTAGS = 1;    // Number of required AprilTags to update the AprilTag estimator
    private final double MAX_YAW_RATE = 720;     // Maximum angular velocity(degrees/s) to update AprilTag estimator

    // Standard deviations(trust values) for encoders and April Tags
    // The lower the numbers the more trustworthy the prediction from that source is
    private final Vector<N3> ENCODER_STD_DEV = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));
    private final Vector<N3> APRILTAG_STD_DEV = VecBuilder.fill(0.7, 0.7, Double.MAX_VALUE);

    // Estimators
    public SwerveDriveOdometry encoderEstimator;        // Might not need this as aprilTagsEstimator also uses the encoders
    public SwerveDrivePoseEstimator aprilTagsEstimator;
    
    public Drive drive;

    public Odometry(Drive drive) {
        this.drive = drive;
        
        SwerveModulePosition[] initialPosition =  getAllModulePositions();
        Rotation2d initialRotation = new Rotation2d(drive.getYawAdjusted());

        encoderEstimator = new SwerveDriveOdometry(
            drive.swerveDriveKinematics, 
            initialRotation, 
            initialPosition, 
            new Pose2d(0, 0, new Rotation2d(0))
        );

        aprilTagsEstimator = new SwerveDrivePoseEstimator(
            drive.swerveDriveKinematics, 
            initialRotation, 
            initialPosition, 
            new Pose2d(0, 0, new Rotation2d(0)), 
            ENCODER_STD_DEV, 
            APRILTAG_STD_DEV
        );
    }
    
    /**
     * </p> Updates the pose estimators
     */
    public void update() {
        SwerveModulePosition[] currentPosition = getAllModulePositions();
        Rotation2d currentRotation = new Rotation2d(drive.getYawAdjusted());
        
        // Encoder Estimator
        encoderEstimator.update(currentRotation, currentPosition);

        // AprilTags Estimator
        // This function assumes rotation is CCW-positive and 0 degrees/radians when facing the red alliance wall
        LimelightHelpers.SetRobotOrientation(
            "limelight", 
            drive.getYawDegreesAdjusted(), 
            0, 0, 0, 0, 0   // These are always 0
        );

        // Get position, always use the blue position because wpilib moved to a single-origin system
        LimelightHelpers.PoseEstimate megatags2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        // Add AprilTag estimates
        if(drive.getYawDegreesRate() < MAX_YAW_RATE && megatags2Pose.tagCount >= REQUIRED_APRILTAGS) {
            aprilTagsEstimator.addVisionMeasurement(megatags2Pose.pose, megatags2Pose.timestampSeconds);
        }
        aprilTagsEstimator.update(currentRotation, currentPosition);
    }

    /**
     * </p> Gets the current encoder estimated position
     * 
     * @return The estimated Pose with the encoders (in meters)
     */
    public Pose2d getEncoderPose() {
        return encoderEstimator.getPoseMeters();
    }

    /**
     * </p> Gets the current AprilTags-assisted estimated position
     * 
     * @return The estimated Pose with the AprilTags (in meters)
     */
    public Pose2d getAprilTagsPose() {
        return aprilTagsEstimator.getEstimatedPosition();
    }

    /**
     * </p> Resets the estimators to a given pose
     * </p> Gyro angle and swerve module positions don't have to be reset beforehand
     *      as the estimators automatically creates offsets
     * 
     * @param newPose The Pose2d to reset to
    */
    public void reset(Pose2d newPose) {
        SwerveModulePosition[] currentPosition = getAllModulePositions();
        Rotation2d gyro = new Rotation2d(drive.getYawAdjusted());

        encoderEstimator.resetPosition(
            gyro,
            currentPosition,
            newPose
        );

        aprilTagsEstimator.resetPosition(
            gyro, 
            currentPosition, 
            newPose
        );
    }

    /**
     * <p> Returns the distance to the speaker (feet)
     * @param isRed If the robot is on the red alliance
     * @return The distance to the speaker
     */
    public double distanceToSpeaker(boolean isRed) {
        Pose2d poseMeters = aprilTagsEstimator.getEstimatedPosition();

        // Conver current pose to feet instead of meters
        Pose2d poseFeet = new Pose2d(poseMeters.getX() * 3.28084, poseMeters.getY() * 3.28084, poseMeters.getRotation());

        /* (0,0) is always next to the red alliance note feed
         * Where X+ is towards the red side
         * Where Y+ is towards the amp (either side)
         * From field docs, speaker-center AprilTag positions, divide by 12 to get feet */
        final Translation2d BLUE_SPEAKER_POSE = new Translation2d(-1.50 / 12, 218.42 / 12);
        final Translation2d RED_SPEAKER_POSE = new Translation2d(652.73 / 12, 218.42 / 12);

        Translation2d speaker = (isRed) ? RED_SPEAKER_POSE : BLUE_SPEAKER_POSE;
        
        return poseFeet.getTranslation().getDistance(speaker);
    }

    /*******************************************************************************************
     *
     *                                     HELPER FUNCTIONS
     * 
     *******************************************************************************************/

    /**
     * Gets the position of all four SwerveModules.
     * 
     * @return The position of all four wheels
     */
    private SwerveModulePosition[] getAllModulePositions() {
        return new SwerveModulePosition[] {
            drive.getFLPosition(),
            drive.getFRPosition(),
            drive.getBLPosition(),
            drive.getBRPosition()
        };
    }

    /*******************************************************************************************
     *
     *                                      TEST FUNCTIONS
     * 
     *******************************************************************************************/
    /**
     * <p> Prints the current estimated pose from the encoders
    */
     public void printEncoderEstimatorPose() {
        printPose(getEncoderPose());
    }
    /**
     * <p> Prints the current estimated pose from the AprilTags
    */
    public void printAprilTagEstimatorPose() {
        printPose(getAprilTagsPose());
    }
    /**
     * <p> Prints the given pose
    */
    public void printPose(Pose2d pose) {
        Rotation2d rotation = pose.getRotation();

        System.out.println(
            "Field Position: (" + pose.getX() + ", " + pose.getY() + ")\n" +
            "Field Rotation: (" + rotation.getDegrees() + "deg, " + rotation.getRadians() + "rad"
        );
    }
}
