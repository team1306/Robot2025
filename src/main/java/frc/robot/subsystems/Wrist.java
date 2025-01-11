package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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

public class Wrist extends SubsystemBase  {

    private final TalonFX leftArmMotor, rightArmMotor;
    private final DutyCycleEncoder absoluteThroughBoreEncoder;


    private final PIDController pidController;

    @GetValue public static double kP = 0, kI = 0.0, kD = 0.00; 
    private static final double MAX_RPS = 360;
    private double maxPower = 1;

    public static final double OFFSET = -1, DELTA_AT_SETPOINT = .2;
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);


    public Wrist() {
        super("arm");
        leftArmMotor = MotorUtil.initTalonFX(WRIST_LEFT_MOTOR_ID, NeutralModeValue.Brake);
        rightArmMotor = MotorUtil.initTalonFX(WRIST_RIGHT_MOTOR_ID, NeutralModeValue.Brake);
        
        absoluteThroughBoreEncoder = new DutyCycleEncoder(WRIST_ABSOLUTE_ENCODER);

        pidController = new PIDController(kP, kI, kD, LOOP_TIME_SECONDS);
        pidController.setTolerance(DELTA_AT_SETPOINT);

        setTargetAngle(getCurrentAngle());
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((absoluteThroughBoreEncoder.get())).minus(Rotation2d.fromDegrees(OFFSET));
    }

    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(Rotation2d angle) {
        targetAngle = angle;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    @Override
    public void periodic() {
    
    }
}