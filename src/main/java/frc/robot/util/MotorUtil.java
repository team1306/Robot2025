package frc.robot.util;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.NEO_CURRENT_LIMIT_AMPS;

public class MotorUtil {
    public static SparkMax initSparkMax(int motorId, MotorType motorType, IdleMode idleMode, int currentLimitAmps) {
        SparkMax motor = new SparkMax(motorId, motorType);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        config.smartCurrentLimit(currentLimitAmps);
        motor.configure(config, null, PersistMode.kPersistParameters);
        return motor;
    }

    /**
     * Create a new CANSparkMax Neo motor
     * @param motorId the CAN id of the motor
     * @param motorType the type of the motor (Brushed or Brushless)
     * @param idleMode the idle mode of the motor (Brake or Coast)
     * @return the motor with the paramenters specified
     */
    public static SparkMax initSparkMax(int motorId, MotorType motorType, IdleMode idleMode) {
        return initSparkMax(motorId, motorType, idleMode, NEO_CURRENT_LIMIT_AMPS);
    }

    /**
     * Clamp the percent output to be between -1 and 1  
     * @return the clamped result
     */
    public static double clampPercent(double percent){
        return MathUtil.clamp(percent, -1, 1);
    }
    
}
