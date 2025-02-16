package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.GetValue;
import frc.robot.util.Dashboard.PutValue;
import lombok.Getter;
import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase  {

    @GetValue 
    public double kP = 0.4, kI = 0.005, kD = 0.01, kG = 0.02;

    private final double MIN_ANGLE = 0, MAX_ANGLE = 100;

    private final Rotation2d OFFSET = Rotation2d.fromDegrees(47.08);
    private final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0.1);

    @Getter @PutValue
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    @PutValue
    public Rotation2d currentAngle;

    private final TalonFX motor;
    private final TalonFXGroup motorGroup;

    private final DutyCycleEncoder encoder;
    private final PIDController pidController;

    private ArmFeedforward feedforward;

    /**
     * The wrist is mounted on the arm and rotates the intake to place and pick up coral.
     * Hardware: The wrist has one Talon FX motor controller and an Absolute Analog Encoder.
     * Controllers: Normal PID Controller.
     */
    public Wrist() {
        DashboardHelpers.addUpdateClass(this);
        
        motor = MotorUtil.initTalonFX(WRIST_MOTOR_ID, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive);
        motorGroup = new TalonFXGroup(new TalonData(motor));
        encoder = new DutyCycleEncoder(WRIST_ENCODER_ID);

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(TOLERANCE.getRadians());
        
        setTargetAngle(Rotation2d.kZero);
    }

    @Override
    public void periodic() {
        feedforward = new ArmFeedforward(0, kG, 0, 0);
        pidController.setPID(kP, kI, kD);
        
        currentAngle = getCurrentAngle();
        double pidOutput = pidController.calculate(currentAngle.getRadians(), targetAngle.getRadians());
        double feedforwardOutput = feedforward.calculate(currentAngle.getRadians(), 0);

        double motorOutput = pidOutput + feedforwardOutput;
        motorGroup.setSpeed(motorOutput);
    }

    /**
     * Gets the current angle of the wrist.
     * @return the rotation of the wrist.
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((encoder.get())).minus(OFFSET).unaryMinus();
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