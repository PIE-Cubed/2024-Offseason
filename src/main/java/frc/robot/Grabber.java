package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Grabber {
    private final int GRABBER_MOTOR1_CAN = 2;
    private final int GRABBER_MOTOR2_CAN = 3;
    private final int GRABBER_MOTOR3_CAN = 4;
    private final int INTAKE_BUTTONS_ID = 0;

    private final boolean GRABBER_MOTOR1_IS_INVERTED = true;
    private final boolean GRABBER_MOTOR2_IS_INVERTED = false;
    private final boolean GRABBER_MOTOR3_IS_INVERTED = false;

    // TODO tune these power values
    public final int INTAKE_CURRENT_LIMIT = 40;
    public final double INTAKE_POWER = 0.6;
    public final double AUTO_INTAKE_POWER = 0.6;
    public final double OUTTAKE_POWER = -0.75;
    public final double FEED_POWER = 0.9;  
    public final double PROXIMITY_THRESHOLD = 72;    //95
    public final double IR_THRESHOLD = 150;
    
    private CANSparkMax grabberMotor1;
    private CANSparkMax grabberMotor2;
    private CANSparkMax grabberMotor3;
    private ColorSensorV3 colorSensor;
    private DigitalInput IntakeStopButtons;
    private static Grabber instancedGrabber;
    private DigitalInput irSensor;


    public static synchronized Grabber getInstance() {
        //System.out.println("[INFO] >> Initializing instanced grabber...");

        if (instancedGrabber == null){
            instancedGrabber = new Grabber();
        }
        
        return instancedGrabber;
    }
    
    // Constructor
    private Grabber() {        
        //System.out.println("[INFO] >> Initializing grabber motor controllers...");

        // Instantiate grabber motors & sensors
        grabberMotor1 = new CANSparkMax(GRABBER_MOTOR1_CAN, MotorType.kBrushless);
        grabberMotor1.setInverted(GRABBER_MOTOR1_IS_INVERTED);
        grabberMotor1.setSmartCurrentLimit(INTAKE_CURRENT_LIMIT);

        grabberMotor2 = new CANSparkMax(GRABBER_MOTOR2_CAN, MotorType.kBrushless);
        grabberMotor2.setInverted(GRABBER_MOTOR2_IS_INVERTED);
        grabberMotor2.setSmartCurrentLimit(INTAKE_CURRENT_LIMIT);

        grabberMotor3 = new CANSparkMax(GRABBER_MOTOR3_CAN, MotorType.kBrushed);
        grabberMotor3.setInverted(GRABBER_MOTOR3_IS_INVERTED);
        grabberMotor3.setSmartCurrentLimit(INTAKE_CURRENT_LIMIT);

        // Set the chassis grabber motors to coast, so the note won't get caught
        // Set the arm grabber motor to brake to make sure the not doesn't go too far
        grabberMotor1.setIdleMode(IdleMode.kCoast);
        grabberMotor2.setIdleMode(IdleMode.kCoast);
        grabberMotor3.setIdleMode(IdleMode.kBrake);
        
        //System.out.println("[INFO] >> Initializing grabber sensors...");
       // colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

        // Create the digital input for the intake buttons
       // IntakeStopButtons = new DigitalInput(INTAKE_BUTTONS_ID);

        irSensor = new DigitalInput(3);
    } 


    /**
     * Sets the grabber motor's power
     * 
     * @param power
     */
    public void setMotorPower(double power) {
        grabberMotor1.set(MathUtil.clamp(-1 * power, -1, 1));
        grabberMotor2.set(MathUtil.clamp(power, -1, 1));
        grabberMotor3.set(MathUtil.clamp(power * 1.5, -1, 1));  // 775 seems to need more power
    }

    /**
     * <p> Sets the motor power to feed power
     * <p> Cuts power when a note is detected
     * 
     * @return Robot status, CONT if there is no note, DONE if there is
     * 
     */
    // TODO test color sensor's ability to stop intake correctly
    /*
     * Mr McMahon comments to be deleted
     *   I don't think isAuto does anything and can be removed
     */
    public int intakeOutake(boolean intake, boolean outtake, boolean isAuto) {
        if (intake && outtake) {
            return Robot.CONT;
        }

        if(intake) {
            if(noteDetected()) {
                setMotorPower(0.0);
                
                return Robot.DONE;
            } 
            else {
                if(isAuto) {
                    setMotorPower(AUTO_INTAKE_POWER);
                }
                else {                    
                    setMotorPower(INTAKE_POWER);
                }

                return Robot.CONT;
            }
        } 
        else if(outtake) {
            setMotorPower(OUTTAKE_POWER);
            return Robot.CONT;
        }
        else {
            setMotorPower(0.0);
            return Robot.DONE;
        }
    }

    /**
     * <p> Checks if the the color sensor senses a note
     * <p> Uses a threshold to determine when to cut power
     * <p> Initial values are ~110 without a note, and ~250 with a note 
     * @return whether a note is found
     */
    public boolean noteDetected() {
     //   return !IntakeStopButtons.get() || colorSensor.getProximity() > PROXIMITY_THRESHOLD;
        return !irSensor.get();
    }

    /**
     * Prints color(r,g,b), IR, and proximity readings from the sensor
     * IR has a range; we've observed values from 110 with no note, to 250 with note
     * Red goes from 0.28 with no note to 0.38 with note
     *
     * IR works best due to its high range/delta, and is more tolerant 
     * to environmental interference from sources such as light
     */
    public void testColorSensor() {
        Color detectedColor = colorSensor.getColor();
        
        double r = 255 * detectedColor.red;
        double g = 255 * detectedColor.green;
        double b = 255 * detectedColor.blue;
        double ir = colorSensor.getIR();
        double proximity = colorSensor.getProximity();
        
        System.out.println( "[== COLOR SENSOR ==]" +
            "\n | R: "  + r  +
            "\n | G: "  + g  +
            "\n | B: "  + b  +
            "\n | IR: " + ir +
            "\n | Proximity: " + proximity + "\n\n"
        );
    }

    public double getProximity() {
        return colorSensor.getProximity();
    }

    public double getPower1() {
        return grabberMotor1.get();
    }

    public double getPower2() {
        return grabberMotor2.get();
    }
    
    public double getPower3() {
        return grabberMotor3.get();
    }

    public void testIRsensor()  {
        boolean ringInView;

        ringInView = irSensor.get();
        System.out.println("ring in view:" + !ringInView);
    }
}
