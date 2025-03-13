package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.*;
import frc.robot.util.MotorUtil;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import lombok.Getter;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase  {
    
    @Entry(type = EntryType.Subscriber)
    private static double kG = 0.033, kV = 0;
    
    private static final double MAX_VELOCITY = Double.MAX_VALUE, MAX_ACCELERATION = Double.MAX_VALUE;
    
    private final double MIN_ANGLE = -30, MAX_ANGLE = 90;

    private final Rotation2d OFFSET = Rotation2d.fromDegrees(52.3 + 2.8), TOLERANCE = Rotation2d.fromDegrees(0.1);

    @Getter @Entry(type = EntryType.Publisher)
    private static Rotation2d targetAngle = Rotation2d.kZero;

    @Entry(type = EntryType.Publisher)
    public static Rotation2d currentAngle = Rotation2d.kZero;

    private final MotorGroup<Motor> motorGroup;
    private final DutyCycleEncoder armEncoder;

    @Entry(type = EntryType.Sendable)
    private static ProfiledPIDController profiledPIDController = new ProfiledPIDController(0.015, 0, 0.0008, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    private ArmFeedforward feedforward;


    private final DetectUnpluggedEncoder detectEncoderUnplugged;
    private final Alert encoderUnpluggedAlert = new Alert("Arm encoder detected unplugged", AlertType.kError);
    
    /**
     * The arm is mounted on the elevator and moves the wrist and intake to place and pick up coral.
     * Hardware: The arm has one Talon FX motor controller and an Absolute Analog Encoder
     * Controllers: Feedforward and Profiled PID Controller.
     */
    public Arm() {
//        Motor motor = new TalonFxMotor(MotorUtil.initTalonFX(ARM_MOTOR_ID, NeutralModeValue.Brake));
        Motor motor = new FakeMotor();
        motorGroup = new MotorGroup<>(motor);

        armEncoder = new DutyCycleEncoder(ARM_ENCODER_ID);

        feedforward = new ArmFeedforward(0, kG, kV, 0);

        profiledPIDController.setTolerance(TOLERANCE.getDegrees());
        setTargetAngle(Rotation2d.kZero);

        detectEncoderUnplugged = new DetectUnpluggedEncoder(() -> getCurrentAngle().getDegrees(), () -> targetAngle.getDegrees());
    }

    @Override
    public void periodic() {
        feedforward = new ArmFeedforward(0, kG, kV, 0);

        double pidOutput = profiledPIDController.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees());
        final State state = profiledPIDController.getSetpoint();
        double feedforwardOutput = feedforward.calculate(getCurrentAngle().getRadians(), Math.toRadians(state.velocity));
        double motorOutput = pidOutput + feedforwardOutput;

        currentAngle = getCurrentAngle();

        motorGroup.setSpeed(motorOutput);

        if (detectEncoderUnplugged.update()) encoderUnpluggedAlert.set(true);
        else encoderUnpluggedAlert.set(false);
    }

    /**
     * Gets if the arm is at its setpoint.
     * @return true if the arm is at its setpoint (with a tolerance of course).
     */
    public boolean atSetpoint() {
        return Math.abs(currentAngle.minus(targetAngle).getDegrees()) < TOLERANCE.getDegrees();
    }

    /**
     * Gets the current angle of the arm.
     * @return the rotation of the arm.
     */
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(armEncoder.get()).minus(OFFSET);
    }

    /**
     * Sets the target angle of the arm.
     * @param setpoint the rotation for the arm setpoint.
     */
    public void setTargetAngle(Rotation2d setpoint) {
        targetAngle = Rotation2d.fromRadians(MathUtil.clamp(setpoint.getRadians(), MIN_ANGLE, MAX_ANGLE));
    }
}