package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorUtil;

public class Elevator extends SubsystemBase {
    
    // go to and hold positions (thrifty elevator) [PID and Feedforward]

    private final double PID_P = 1, PID_I = 0, PID_D = 0;
    private PIDController pid = new PIDController(PID_P, PID_I, PID_D);

    private final double FF_S = 0, FF_G = 0, FF_V = 0;
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(FF_S, FF_G, FF_V);

    private final SparkMax leftMotor = MotorUtil.initSparkMax(Constants.ELEVATOR_LEFT_MOTOR_ID, IdleMode.kBrake),
                           rightMotor = MotorUtil.initSparkMax(Constants.ELEVATOR_RIGHT_MOTOR_ID, IdleMode.kBrake);

    public Elevator() {

    }
}
