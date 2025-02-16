package frc.robot.util;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.NEO_CURRENT_LIMIT_AMPS;

public class MotorUtil {
    public static SparkMax initSparkMax(int motorId, IdleMode idleMode, boolean inverted) {
        SparkMax motor = new SparkMax(motorId, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(idleMode);
        config.smartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
        config.inverted(inverted);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        return motor;
    }

    public static SparkMax initSparkMax(int motorId, IdleMode idleMode) {
        return initSparkMax(motorId, idleMode, false);
    }

    public static TalonFX initTalonFX(int motorId, NeutralModeValue idleMode){
        return initTalonFX(motorId, idleMode, InvertedValue.Clockwise_Positive);
    }

    public static TalonFX initTalonFX(int motorId, NeutralModeValue idleMode, InvertedValue inverted){
        final TalonFX motor = new TalonFX(motorId);
        final TalonFXConfigurator configurator = motor.getConfigurator();

        final MotorOutputConfigs motorOutputConfig = new MotorOutputConfigs();

        motorOutputConfig.Inverted = inverted;
        motorOutputConfig.NeutralMode = idleMode;
   
        configurator.apply(motorOutputConfig);

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
