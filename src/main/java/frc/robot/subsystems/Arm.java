package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.GetValue;
import lombok.Getter;
import lombok.Setter;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase  {

    private final TalonFX leftMotor, rightMotor;
    private final TalonFXGroup motorGroup;
    private final DutyCycleEncoder armEncoder;

    @GetValue public double kP = 0, kI = 0, kD = 0; 
    @GetValue public double kG = 0, kV = 0; 
    private final double MAX_VELOCITY = Double.MAX_VALUE;
    private final double MAX_ACCELERATION = Double.MAX_VALUE;

    private final ProfiledPIDController profiledPIDController;
    private ArmFeedforward feedforward;

    private final Rotation2d OFFSET = Rotation2d.kZero;
    private final Rotation2d TOLERANCE = Rotation2d.kZero;

    @Getter @Setter
    private Rotation2d targetAngle = Rotation2d.kZero;

    public Arm() {
        leftMotor = MotorUtil.initTalonFX(ARM_LEFT_MOTOR_ID, NeutralModeValue.Brake);
        rightMotor = MotorUtil.initTalonFX(ARM_RIGHT_MOTOR_ID, NeutralModeValue.Brake);
        
        motorGroup = new TalonFXGroup(new TalonData(leftMotor), new TalonData(rightMotor));

        armEncoder = new DutyCycleEncoder(0);

        feedforward = new ArmFeedforward(0, kG, kV, 0);
        profiledPIDController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        profiledPIDController.setTolerance(TOLERANCE.getRadians());
        setTargetAngle(getCurrentAngle());
    }

    @Override
    public void periodic() {
        profiledPIDController.setPID(kP, kI, kD);
        feedforward = new ArmFeedforward(0, kG, kV, 0);

        double pidOutput = profiledPIDController.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees());

        final State state = profiledPIDController.getSetpoint();
        double feedforwardOutput = feedforward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));

        double motorOutput = pidOutput + feedforwardOutput;
        motorGroup.setSpeed(motorOutput);
    }

    public boolean atSetpoint() {
        return Math.abs(getCurrentAngle().minus(targetAngle).getDegrees()) < TOLERANCE.getDegrees();
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(armEncoder.get()).minus(OFFSET);
    }
}