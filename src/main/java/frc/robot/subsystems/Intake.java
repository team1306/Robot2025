package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.MotorGroup;
import frc.robot.subsystems.utils.TalonFxMotor;
import frc.robot.util.MotorUtil;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import lombok.Getter;
import static frc.robot.Constants.INTAKE_MOTOR_ID;
import static frc.robot.Constants.INTAKE_SENSOR_ID;

public class Intake extends SubsystemBase {

    @Getter
    private double targetSpeed = 0;

    @Entry(type = EntryType.Publisher)
    private static boolean sensorReading = false;
    
    private final TalonFX motor;
    private final MotorGroup<TalonFxMotor> motorGroup;
    private final DigitalInput sensor;

    /**
     * The intake is mounted on the wrist and used to pick up coral and release it to score.
     * Hardware: The intake has one Talon FX motor controller and a beam-break sensor.
     * Controllers: None
     */
    public Intake() {        
        motor = MotorUtil.initTalonFX(INTAKE_MOTOR_ID, NeutralModeValue.Coast);
        motorGroup = new MotorGroup<>(new TalonFxMotor(motor));
        sensor = new DigitalInput(INTAKE_SENSOR_ID);
    }

    @Override
    public void periodic() {
        motorGroup.setSpeed(targetSpeed);
        sensorReading = sensor.get();
    }

    /**
     * Sets the target speed of the intake.
     * @param targetSpeed the speed for the intake setpoint (-1 - 1).
     */
    public void setTargetSpeed(double targetSpeed) {
        this.targetSpeed = MotorUtil.clampPercent(targetSpeed);
    }

    /**
     * Gets the reading from the beam-break sensor on the intake.
     * @return true if the sensor detects an object in the intake
     */
    public boolean getSensorReading() {
        return sensorReading;
    }
}