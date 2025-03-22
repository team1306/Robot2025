package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.Motor;
import frc.robot.subsystems.utils.MotorGroup;
import frc.robot.subsystems.utils.TalonFxMotor;
import frc.robot.util.MotorUtil;
import lombok.Getter;
import lombok.Setter;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {
    private final MotorGroup<Motor> motorGroup;

    @Getter @Setter
    private double targetSpeed;

    public Climber() {
        Motor motor = new TalonFxMotor(MotorUtil.initTalonFX(CLIMB_MOTOR_ID, NeutralModeValue.Brake));
//        Motor motor = new FakeMotor();

        motorGroup = new MotorGroup<>(motor);
    }

    @Override
    public void periodic() {
        motorGroup.setSpeed(targetSpeed);
    }
}
