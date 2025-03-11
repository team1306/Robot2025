package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.MotorGroup;
import frc.robot.subsystems.utils.TalonFxMotor;
import frc.robot.util.MotorUtil;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import lombok.Getter;
import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase  {
    private final double MIN_ANGLE = -100, MAX_ANGLE = 100;

    private final Rotation2d OFFSET = Rotation2d.fromDegrees(266.5);
    
    @Entry(type = EntryType.Subscriber)
    private static double offsetRight = 0D;

    private final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0);

    @Getter @Entry(type = EntryType.Publisher)
    private static Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    @Entry(type = EntryType.Publisher)
    public static Rotation2d currentAngle = Rotation2d.fromDegrees(0);

    private TalonFX motor;
    private MotorGroup<TalonFxMotor> motorGroup;

    private  DutyCycleEncoder encoder;

    @Entry(type = EntryType.Sendable)
    private static PIDController pidController = new PIDController(0.5, 0, 0.015);

    /**
     * The wrist is mounted on the arm and rotates the intake to place and pick up coral.
     * Hardware: The wrist has one Talon FX motor controller and an Absolute Analog Encoder.
     * Controllers: Normal PID Controller.
     */
    public Wrist() {        
        motor = MotorUtil.initTalonFX(WRIST_MOTOR_ID, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
        motorGroup = new MotorGroup<>(new TalonFxMotor(motor));
        encoder = new DutyCycleEncoder(WRIST_ENCODER_ID);

        pidController.setTolerance(TOLERANCE.getRadians());
        
        setTargetAngle(Rotation2d.kZero);
    }

    @Entry(type = EntryType.Publisher)
    private static double motorOutput = 0;

    @Override
    public void periodic() {        
        currentAngle = getCurrentAngle();
        double pidOutput = pidController.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        motorOutput = pidOutput;
        motorGroup.setSpeed(motorOutput);
    }

    /**
     * Gets the current angle of the wrist.
     * @return the rotation of the wrist.
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((encoder.get())).minus(OFFSET).unaryMinus().plus(Rotation2d.fromDegrees(offsetRight));
    }

    /**
     * Gets if the arm is at its setpoint using the PID controller.
     * @return true if the arm is at its setpoint.
     */
    public boolean atSetpoint() {
        return Math.abs(currentAngle.minus(targetAngle).getDegrees()) < TOLERANCE.getDegrees(); 
    }

    /**
     * Sets the target angle of the wrist.
     * @param setpoint the rotation for the wrist setpoint.
     */
    public void setTargetAngle(Rotation2d setpoint) {
        targetAngle = Rotation2d.fromRadians(MathUtil.clamp(setpoint.getRadians(), MIN_ANGLE, MAX_ANGLE));
    }
}