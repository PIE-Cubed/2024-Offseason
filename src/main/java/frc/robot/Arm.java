package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;

public class Arm {
    private final int ELEVATION_MOTOR_CAN = 1;
    private final int MOTOR_CURRENT_LIMIT = 40;
    
    private final double LOWER_ELEVATION_LIMIT = 0;
    private final double UPPER_ELEVATION_LIMIT = Math.PI;
    
    private final double ELEVATION_TOLERANCE_DEGREES = 2;
    
    private final double ELEVATION_ENCODER_FACTOR = 360;
    
    private final double ARM_ELEVATION_PID_P = 0.04;
    private final double ARM_ELEVATION_PID_I = ARM_ELEVATION_PID_P / 1.5;
        
    public final double ARM_REST_POSITION_DEGREES = 329;
    public final double ARM_AMP_POSITION_DEGREES = 25.65;

    private CANSparkMax elevationMotor;
    
    private AbsoluteEncoder elevationEncoder;
    
    private PIDController elevationMotorPidController;
    
    private DigitalInput restStopButton;
    private DigitalInput intake1StopButton;
    private DigitalInput intake2StopButton;
    
    private int rotateTargetCount = 0;
    
    //public final double ARM_POT_REST_POSITION = 58;
    //public final double ARM_POT_INTAKE_POSITION = 135;

    private boolean elevationFirstTime;
    private double elevationAngle;

    private double startPosition;
    private double targetDistance;

    private double rotateStatus = Robot.CONT;

    public Arm() {        
        //System.out.println("[INFO] >> Initializing arm motors...");
        
        // Setup elevation motor
        elevationMotor = new CANSparkMax(ELEVATION_MOTOR_CAN, MotorType.kBrushless);
        elevationMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        elevationMotor.setIdleMode(IdleMode.kBrake);

        // Elevation Absolute
        elevationEncoder = elevationMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elevationEncoder.setPositionConversionFactor(ELEVATION_ENCODER_FACTOR);
        elevationEncoder.setVelocityConversionFactor(ELEVATION_ENCODER_FACTOR);

        // PID controllers
        //System.out.println("[INFO] >> Initializing arm PID controllers...");

        // Elevation PID
        elevationMotorPidController = new PIDController(ARM_ELEVATION_PID_P, ARM_ELEVATION_PID_I, 0.0);
        elevationMotorPidController.setIntegratorRange(-0.1, 0.1);
        elevationMotorPidController.setTolerance(ELEVATION_TOLERANCE_DEGREES);
        elevationMotorPidController.enableContinuousInput(0, 360);

        elevationFirstTime = true;
        elevationAngle = 0.0;

        startPosition = elevationEncoder.getPosition();
        //System.out.println("[INFO] >> Arm start position: " + startPosition);
    }

    /**
     * <p>Rotates the arm with the given radian value
     * <p>0 degrees is horizontal
     * 
     * @param degrees The angle to rotate to in degrees
     * @return Robot.CONT or Robot.DONE
     */
    public int rotateArm(double degrees) {
        if(elevationFirstTime) {
            elevationFirstTime = false;
            elevationMotorPidController.setSetpoint(degrees);
            rotateTargetCount = 0;
        }

        //System.out.println("From: " + elevationEncoder.getPosition() + " To: " + degrees);
        /* Negative power moves the arm upward;
            The PID value will be positive to increase the angle */
        double power = -elevationMotorPidController.calculate(elevationEncoder.getPosition(), degrees);
        //SmartDashboard.putNumber("Arm power", power);
        elevationMotor.set(MathUtil.clamp(power, -0.3, 0.3)); // Clamp
                
        if(elevationMotorPidController.atSetpoint()) {
            rotateTargetCount++;

            if (rotateTargetCount >= 5) {
                elevationFirstTime = true;
                return Robot.DONE;
            }
        }
        else{
            rotateTargetCount = 0;
        }

        return Robot.CONT;
    }

    public int rotateToRest(double powerMultiplier) {
        if(elevationFirstTime) {
            elevationFirstTime = false;
            elevationMotorPidController.setSetpoint(ARM_REST_POSITION_DEGREES);
            rotateTargetCount = 0;
        }

        //System.out.println("From: " + elevationEncoder.getPosition() + " To: " + degrees);
        /* Negative power moves the arm upward;
            The PID value will be positive to increase the angle */
        double power = -elevationMotorPidController.calculate(elevationEncoder.getPosition(), ARM_REST_POSITION_DEGREES);
        power *= powerMultiplier;
        //SmartDashboard.putNumber("Arm power", power);
        elevationMotor.set(MathUtil.clamp(power, -0.3, 0.3)); // Clamp
                
        if(elevationMotorPidController.atSetpoint()) {
            rotateTargetCount++;

            if (rotateTargetCount >= 5) {
                elevationFirstTime = true;
                //elevationMotor.set(0.1);
                return Robot.DONE;
            }
        }
        else{
            rotateTargetCount = 0;
        }

        return Robot.CONT;
    }

    /**
     * <p>Keeps the arm at its current position
     * <p>0 degrees is horizontal
     * 
     * @param degrees The angle to rotate to in degrees
     * @return Robot.CONT or Robot.DONE
     */
    public int maintainPosition(double degrees) {                
        /* Negative power moves the arm upward;
            The PID value will be positive to increase the angle */
        double power = -elevationMotorPidController.calculate(elevationEncoder.getPosition(), degrees);
        elevationMotor.set(MathUtil.clamp(power, -0.4, 0.4));   // Clamp                
        return Robot.CONT;
    }

    /**************************************************************************
     * 
     *      ACTION FUNCTIONS
     * 
     **************************************************************************/

    /**
     * Rotates arm incrementaly up
     * 
     * @param rotateUp
     * @param rotateDown
     */
    public void rotateArmIncrement(boolean rotateUp, boolean rotateDown) {
        if(rotateUp){
            elevationAngle += 0.75;
        }
        else if(rotateDown) {
            elevationAngle -= 0.75;
        } 
        else {
            elevationAngle += 0;
        }

        elevationAngle = MathUtil.clamp(elevationAngle, LOWER_ELEVATION_LIMIT, UPPER_ELEVATION_LIMIT);

        rotateArm(elevationAngle);
    }


    public double getElevationPosition() {
        return elevationEncoder.getPosition();
    }

    /**
     * Turns off the elevation motor to let it fall
     */
    public void disableRotation() {
        elevationMotor.set(0);
    }


    /**************************************************************************
     * 
     *      TEST FUNCTIONS
     * 
     **************************************************************************/

    /// Positive power goes down
    public void testElevate(double power) {
        elevationMotor.set(power);
    }

    /// Put the arm position on shuffleboard
    public void testPosition() {
        System.out.println(elevationEncoder.getPosition());
    }

    /// Move the arm to its starting/balancing position (54 degrees)
    public int testStartingPosition() {
        return rotateArm(54);
    }

    /// Returns the status of intake button 2
    public boolean getIntakeButton1() {
        return !intake1StopButton.get();
    }

    /// Returns the status of intake button 2
    public boolean getIntakeButton2() {
        return !intake2StopButton.get();
    }

    /// Returns the status of the intake buttons
    /// Returns whether either are pressed
    public boolean getIntakeButtons() {
        return !intake1StopButton.get() || !intake2StopButton.get();
    }
    
    /// Returns the status of the intake button
    public boolean getRestButton() {
        return !restStopButton.get();
    }
}
