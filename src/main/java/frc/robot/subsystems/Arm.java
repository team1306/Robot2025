package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.GetValue;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase  {

    private static double a = -9.99e-4, b = 0.408, c = -1.1;
    private final TalonFX leftArmMotor, rightArmMotor;
    private final Encoder relativeThroughBore;
    private final DutyCycleEncoder absoluteThroughBoreEncoder;

    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, maxAcceleration);

    private final ProfiledPIDController profiledPIDController;
    private ArmFeedforward feedforward;

    @GetValue public static double kP = 0.035, kI = 0.02, kD = 0.004; 
    @GetValue public static double kG = 0.0725, kV = .17; 
    private static final double MAX_VELOCITY = 360;
    private static double maxAcceleration = 140; // kMA MIGHT BE WRONG
    private double armMaxPower = 1;

    public static final double OFFSET = 202.925, DELTA_AT_SETPOINT = .6;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    private long velocityIndex = 0;
    private final double[] velocities = new double[2];


    public Arm() {
        super("arm");
        leftArmMotor = MotorUtil.initTalonFX(ARM_LEFT_MOTOR_ID, NeutralModeValue.Brake);
        rightArmMotor = MotorUtil.initTalonFX(ARM_RIGHT_MOTOR_ID, NeutralModeValue.Brake);
        
        
        absoluteThroughBoreEncoder = new DutyCycleEncoder(0);
        relativeThroughBore = new Encoder(2, 3, true, EncodingType.k1X);
        relativeThroughBore.reset();
        relativeThroughBore.setDistancePerPulse(360D/2048D); // DEGREES_PER_REVOLUTION / CYCLES PER REVOLUTION

        feedforward = new ArmFeedforward(0, Math.min(0.3, kG), kV, 0);
        profiledPIDController = new ProfiledPIDController(kP, kI, kD, constraints, LOOP_TIME_SECONDS);

        profiledPIDController.setTolerance(DELTA_AT_SETPOINT);
        setTargetAngle(getCurrentAngle());
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((absoluteThroughBoreEncoder.get() * -1)).minus(Rotation2d.fromDegrees(OFFSET));
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    private int calcVelocityIndex(long index) {
        return (int) (index % velocities.length);
    }

    private double calculateAcceleration() {
        return (velocities[calcVelocityIndex(velocityIndex)] - velocities[calcVelocityIndex(velocityIndex + 1)]) / LOOP_TIME_SECONDS; 
    }

    public boolean atSetpoint() {
        return Math.abs(getCurrentAngle().minus(targetAngle).getDegrees()) < 2.6;
    }

    @Override
    public void periodic() {
        constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, maxAcceleration);
        profiledPIDController.setConstraints(constraints);
        profiledPIDController.setPID(kP, kI, kD);
        
        feedforward = new ArmFeedforward(0, Math.min(0.25, kG), kV, 0);

        velocities[calcVelocityIndex(velocityIndex)] = relativeThroughBore.getRate();

        double pidOutput = profiledPIDController.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees());
        if (Double.isNaN(pidOutput) || Double.isInfinite(pidOutput)) pidOutput = 0;

        final State state = profiledPIDController.getSetpoint();
        double feedforwardOutput = feedforward.calculate(Math.toRadians(state.position), Math.toRadians(state.velocity));
        if (Double.isNaN(feedforwardOutput) || Double.isInfinite(feedforwardOutput)) feedforwardOutput = 0;

        pidOutput += feedforwardOutput;


        final double motorPower =
                MathUtil.applyDeadband(
                    MathUtil.clamp(getCurrentAngle().getDegrees() < 2.5 && getTargetAngle().getDegrees() < 2.5 ? 0 : pidOutput, -armMaxPower, armMaxPower),
                        .005);

        leftArmMotor.set(motorPower);
        rightArmMotor.set(motorPower);
        ++velocityIndex;
    }
}