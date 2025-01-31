package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.GetValue;
import lombok.Getter;
import static frc.robot.Constants.*;

public class Wrist extends SubsystemBase  {

    @GetValue 
    public double kP = 0, kI = 0.0, kD = 0.00;

    private final double MIN_ANGLE = 0, MAX_ANGLE = 60;

    private final Rotation2d OFFSET = Rotation2d.kZero;
    private final Rotation2d TOLERANCE = Rotation2d.kZero;

    @Getter
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    private final TalonFX motor;
    private final TalonFXGroup motorGroup;

    private final DutyCycleEncoder encoder; //change with arm
    private final PIDController pidController;

    public Wrist() {
        DashboardHelpers.addUpdateClass(this);
        
        motor = MotorUtil.initTalonFX(WRIST_MOTOR_ID, NeutralModeValue.Brake);
        motorGroup = new TalonFXGroup(new TalonData(motor));
        encoder = new DutyCycleEncoder(WRIST_ENCODER_ID);

        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(TOLERANCE.getRadians());

        setTargetAngle(getCurrentAngle());
    }

    @Override
    public void periodic() {
        double pidOutput = pidController.calculate(getCurrentAngle().getRadians(), targetAngle.getRadians());

        //TODO make sure to create a way to override this forced stopping of wrist
        if(getCurrentAngle().getDegrees() < MIN_ANGLE || getCurrentAngle().getDegrees() > MAX_ANGLE)
            pidOutput = 0;
        motorGroup.setSpeed(pidOutput);
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations((encoder.get())).minus(OFFSET);
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public void setTargetAngle(Rotation2d setpoint) {
        targetAngle = Rotation2d.fromRadians(MathUtil.clamp(setpoint.getRadians(), MIN_ANGLE, MAX_ANGLE));
    }
}