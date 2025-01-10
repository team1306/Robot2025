package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lombok.Getter;
import lombok.Setter;

public class Wrist extends SubsystemBase {

    private final double PID_P = 1, PID_I = 0, PID_D = 0; //needs tuning

    private TalonFX motor = new TalonFX(Constants.WRIST_MOTOR_ID);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    private PIDController controller = new PIDController(PID_P, PID_I, PID_D);

    @Getter @Setter
    private Rotation2d angleSetpoint;
    // control falcon 500
    // know theres a gear ratio
    // have pid constants
    // has rev absolute analog encoder to know position
    // be able to set angle in degrees or radians
    public Wrist() {

    }

    @Override
    public void periodic() {
        motor.set(controller.calculate(angleSetpoint.getRadians()));
    }
}
