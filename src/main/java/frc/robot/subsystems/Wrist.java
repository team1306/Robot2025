package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.GetValue;
import lombok.Getter;
import lombok.Setter;

import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase  {

    private final TalonFX motor;
    private final TalonFXGroup motorGroup;

    private final DutyCycleEncoder encoder;
    private final PIDController pidController;

    @GetValue 
    public double kP = 0, kI = 0.0, kD = 0.00; 

    private final Rotation2d OFFSET = Rotation2d.kZero;
    private final Rotation2d TOLERANCE = Rotation2d.kZero;

    @Getter @Setter
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public Wrist() {
        motor = MotorUtil.initTalonFX(WRIST_MOTOR_ID, NeutralModeValue.Brake);
        motorGroup = new TalonFXGroup(new TalonData(motor));
        encoder = new DutyCycleEncoder(WRIST_ABSOLUTE_ENCODER);

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(TOLERANCE.getRadians());

        setTargetAngle(getCurrentAngle());
    }

    @Override
    public void periodic() {
        double pidOutput = pidController.calculate(getCurrentAngle().getRadians(), targetAngle.getRadians());

        motorGroup.setSpeed(pidOutput);
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((encoder.get())).minus(OFFSET);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }
}