package frc.robot;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.CANSparkMax;
public class Climber {
    private final int LEFT_MOTOR_CAN = 50;
    private final int RIGHT_MOTOR_CAN = 51;
    private final int MOTOR_CURRENT_LIMIT = 40;
    
    public final double PRECISION_CLIMB_POWER = 0.2;
    public final double CLIMB_POWER = 0.7;
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;

    public Climber() {
        // Motors
        leftMotor = new CANSparkMax(LEFT_MOTOR_CAN, MotorType.kBrushless);
        leftMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        leftMotor.setIdleMode(IdleMode.kBrake);
        
        rightMotor = new CANSparkMax(RIGHT_MOTOR_CAN, MotorType.kBrushless);
        rightMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }
    /**
     * Runs the left climber motor forwards
     */
    public void runLeftClimberForwards(Boolean on, double power) {
        if (on == true) {
            leftMotor.set(power);
        }
    }
    /**
     * Runs the left climber motor backwards
     */
    public void runLeftClimberBackwards(Boolean on, double power) {
        if (on == true) {
            leftMotor.set(-power);
        }
    }
    /**
     * Runs the right climber motor forwards
     */
    public void runRightClimberForwards(Boolean on, double power) {
        if (on == true) {
            rightMotor.set(power);
        }
    }
    /*
     * Stops left climber
     */
    public void stopLeftClimber() {
        leftMotor.set(0);
    }
    /**
     * Runs the right climber motor backwards
     */
    public void runRightClimberBackwards(Boolean on, double power) {
        if (on == true) {
            rightMotor.set(power);
        }
    }
    /*
     * Stops left climber
     */
    public void stopRightClimber() {
        rightMotor.set(0);
    }
    /**
     * Sets the left motor's power
     * @param power
     */
    public void setLeftClimberPower(double power) {
        leftMotor.set(MathUtil.clamp(power, -1, 1));
    }
    /**
     * Sets the left motor's power
     * @param power
     */
    public void setRightClimberPower(double power) {
        rightMotor.set(MathUtil.clamp(power, -1, 1));
    }
}