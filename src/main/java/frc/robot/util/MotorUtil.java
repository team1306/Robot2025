package frc.robot.util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.NEO_CURRENT_LIMIT_AMPS;

public class MotorUtil {
    public static SparkMax initSparkMax(int motorId, IdleMode idleMode, int currentLimitAmps) {
        SparkMax motor = new SparkMax(motorId, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        config.smartCurrentLimit(currentLimitAmps);
        motor.configure(config, null, PersistMode.kPersistParameters);
        return motor;
    }

    /**
     * Create a new CANSparkMax Neo motor
     *
     * @param motorId   the CAN id of the motor
     * @param motorType the type of the motor (Brushed or Brushless)
     * @param idleMode  the idle mode of the motor (Brake or Coast)
     * @return the motor with the paramenters specified
     */
    public static SparkMax initSparkMax(int motorId, IdleMode idleMode) {
        return initSparkMax(motorId, idleMode, NEO_CURRENT_LIMIT_AMPS);
    }

    public static TalonFX initTalonFX(int motorId, NeutralModeValue idleMode){
        TalonFX motor = new TalonFX(motorId);
        motor.setNeutralMode(idleMode);
        
        return motor;
    }

    /**
     * Clamp the percent output to be between -1 and 1
     *
     * @return the clamped result
     */
    public static double clampPercent(double percent) {
        return MathUtil.clamp(percent, -1, 1);
    }

}
