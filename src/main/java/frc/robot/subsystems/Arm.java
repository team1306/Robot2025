package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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

public class Arm extends SubsystemBase  {
    
    @GetValue public double kP = 0.011, kI = 0, kD = 0.00028; 
    @GetValue public double kG = 0.04, kV = 0; 

    @GetValue
    public boolean manualOverride = false;
    
    private final double MAX_VELOCITY = Double.MAX_VALUE, MAX_ACCELERATION = Double.MAX_VALUE;
    
    private final double MIN_ANGLE = -65, MAX_ANGLE = 80;

    private final Rotation2d OFFSET = Rotation2d.fromDegrees(52.3), TOLERANCE = Rotation2d.kZero;

    @Getter
    private Rotation2d targetAngle = Rotation2d.kZero;

    @PutValue
    public Rotation2d currentAngle = Rotation2d.kZero;

    private final TalonFX motor;
    private final TalonFXGroup motorGroup;
    private final DutyCycleEncoder armEncoder;

    private final ProfiledPIDController profiledPIDController;
    private ArmFeedforward feedforward;

    
    /**
     * The arm is mounted on the elevator and moves the wrist and intake to place and pick up coral.
     * Hardware: The arm has one Talon FX motor controller and an Absolute Analog Encoder
     * Controllers: Feedforward and Profiled PID Controller.
     */
    public Arm() {
        DashboardHelpers.addUpdateClass(this);
        
        motor = MotorUtil.initTalonFX(ARM_MOTOR_ID, NeutralModeValue.Brake);
        
        motorGroup = new TalonFXGroup(new TalonData(motor));

        armEncoder = new DutyCycleEncoder(ARM_ENCODER_ID);

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
        double feedforwardOutput = feedforward.calculate(getCurrentAngle().getRadians(), Math.toRadians(state.velocity));
        double motorOutput = pidOutput + feedforwardOutput;

        currentAngle = getCurrentAngle();
        
        motorGroup.setSpeed(motorOutput);
    }

    /**
     * Gets if the arm is at its setpoint.
     * @return true if the arm is at its setpoint (with a tolerance of course).
     */
    public boolean atSetpoint() {
        return Math.abs(getCurrentAngle().minus(targetAngle).getDegrees()) < TOLERANCE.getDegrees();
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