package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import badgerlog.entry.Entry;
import badgerlog.entry.EntryType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.*;
import frc.robot.util.MotorUtil;
import lombok.Getter;
import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase {
    private final double MIN_ANGLE = -100, MAX_ANGLE = 100;

    private final Rotation2d OFFSET = Rotation2d.fromDegrees(-120 + 25);

    @Entry(EntryType.Subscriber)
    private static double offsetRight = 0D;

    private final Rotation2d TOLERANCE = Rotation2d.fromDegrees(0);

    @Getter
    @Entry(EntryType.Publisher)
    private static Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    @Entry(EntryType.Publisher)
    public static Rotation2d currentAngle = Rotation2d.fromDegrees(0);

    private final MotorGroup<Motor> motorGroup;

    private final DutyCycleEncoder encoder;

    @Entry(EntryType.Sendable)
    private static PIDController pidController = new PIDController(0.27, 0, 0.0001);

    private final DetectUnpluggedEncoder detectEncoderUnplugged;
    private final Alert encoderUnpluggedAlert = new Alert("Arm encoder detected unplugged", AlertType.kError);

    /**
     * The wrist is mounted on the arm and rotates the intake to place and pick up coral.
     * Hardware: The wrist has one Talon FX motor controller and an Absolute Analog Encoder.
     * Controllers: Normal PID Controller.
     */
    public Wrist() {
        Motor motor = new TalonFxMotor(MotorUtil
                .initTalonFX(WRIST_MOTOR_ID, NeutralModeValue.Brake, InvertedValue.Clockwise_Positive));
        // Motor motor = new FakeMotor();

        motorGroup = new MotorGroup<>(motor);
        encoder = new DutyCycleEncoder(WRIST_ENCODER_ID);

        pidController.setTolerance(TOLERANCE.getRadians());

        setTargetAngle(Rotation2d.kZero);

        detectEncoderUnplugged = new DetectUnpluggedEncoder(encoder::get, () -> targetAngle.getDegrees());
    }

    @Entry(EntryType.Publisher)
    private static double motorOutput = 0;

    @Override
    public void periodic() {
        currentAngle = getCurrentAngle();
        double pidOutput = pidController.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        motorOutput = pidOutput;
        motorGroup.setSpeed(motorOutput);

        encoderUnpluggedAlert.set(detectEncoderUnplugged.update());
    }

    /**
     * Gets the current angle of the wrist.
     *
     * @return the rotation of the wrist.
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((encoder.get()))
                .minus(OFFSET)
                .unaryMinus()
                .plus(Rotation2d.fromDegrees(offsetRight));
    }

    /**
     * Gets if the arm is at its setpoint using the PID controller.
     *
     * @return true if the arm is at its setpoint.
     */
    public boolean atSetpoint() {
        return Math.abs(currentAngle.minus(targetAngle).getDegrees()) < TOLERANCE.getDegrees();
    }

    /**
     * Sets the target angle of the wrist.
     *
     * @param setpoint the rotation for the wrist setpoint.
     */
    public void setTargetAngle(Rotation2d setpoint) {
        targetAngle = Rotation2d.fromRadians(MathUtil.clamp(setpoint.getRadians(), MIN_ANGLE, MAX_ANGLE));
    }
}
