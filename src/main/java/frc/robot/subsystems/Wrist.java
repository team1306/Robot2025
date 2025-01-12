package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.GetValue;

import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase  {

    private final TalonFX motor;
    private final DutyCycleEncoder encoder;


    private final PIDController pidController;

    @GetValue public static double kP = 0, kI = 0.0, kD = 0.00; 

    public static final Rotation2d OFFSET = Rotation2d.kZero, PID_TOLERANCE = Rotation2d.fromDegrees(.2);
    
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);


    public Wrist() {
        super("arm");
        motor = MotorUtil.initTalonFX(WRIST_MOTOR_ID, NeutralModeValue.Brake);
        
        encoder = new DutyCycleEncoder(WRIST_ABSOLUTE_ENCODER);

        pidController = new PIDController(kP, kI, kD, LOOP_TIME_SECONDS);
        pidController.setTolerance(PID_TOLERANCE.getRadians());

        setTargetRotation(getCurrentRotation());
    }

    @Override
    public void periodic() {
        double motorPower = pidController.calculate(getCurrentRotation().getRadians(), targetAngle.getRadians());

        motor.set(motorPower);
    }

    public Rotation2d getCurrentRotation() {
        return Rotation2d.fromRotations((encoder.get())).minus(OFFSET);
    }

    public Rotation2d getTargetRotation() {
        return targetAngle;
    }

    public void setTargetRotation(Rotation2d angle) {
        targetAngle = angle;
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
}