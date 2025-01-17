package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.SparkMaxGroup;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;
import frc.robot.subsystems.utils.SparkMaxGroup.SparkMaxData;
import frc.robot.util.MotorUtil;
import lombok.Getter;
import static frc.robot.Constants.INTAKE_MOTOR_ID;
import static frc.robot.Constants.INTAKE_SENSOR_ID;

public class Intake extends SubsystemBase {
    
    private final TalonFX motor;
    private final TalonFXGroup motorGroup;
    private final DigitalInput sensor;

    @Getter
    private double targetSpeed = 0;
    private boolean sensorReading = false;

    public Intake() {
        motor = MotorUtil.initTalonFX(INTAKE_MOTOR_ID, NeutralModeValue.Coast);
        motorGroup = new TalonFXGroup(new TalonData(motor));
        sensor = new DigitalInput(INTAKE_SENSOR_ID);
    }

    @Override
    public void periodic() {
        motorGroup.setSpeed(targetSpeed);
        sensorReading = sensor.get();
        SmartDashboard.putBoolean("Current Intake Sensor Reading", sensorReading);
    }

    public void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }
}