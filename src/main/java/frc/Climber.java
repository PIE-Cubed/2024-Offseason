package frc;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMax;

public class Climber {
    private final int LEFT_MOTOR_CAN = -1;
    private final int RIGHT_MOTOR_CAN = -1;
    private final int MOTOR_CURRENT_LIMIT = 70;
    
    private final double CLIMB_POWER = 0.5;

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
     * Runs the left climber motor
     */
    public void runLeftClimber() {
        leftMotor.set(CLIMB_POWER);
    }

    /**
     * Runs the right climber motor
     */
    public void runRightClimber() {
        rightMotor.set(CLIMB_POWER);
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