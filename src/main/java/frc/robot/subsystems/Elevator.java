package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorUtil;

public class Elevator extends SubsystemBase {
    
    // go to and hold positions (thrifty elevator) [PID and Feedforward]

    private final double PID_P = 1, PID_I = 0, PID_D = 0, 
        MAX_VELOCITY = Double.MAX_VALUE, MAX_ACCELERATION = Double.MAX_VALUE;
    private ProfiledPIDController pid = new ProfiledPIDController(PID_P, PID_I, PID_D, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

    private final double FF_S = 0, FF_G = 0, FF_V = 0;
    private ElevatorFeedforward feedforward = new ElevatorFeedforward(FF_S, FF_G, FF_V);

    private final SparkMax leftMotor = MotorUtil.initSparkMax(Constants.ELEVATOR_LEFT_MOTOR_ID, IdleMode.kBrake),
                           rightMotor = MotorUtil.initSparkMax(Constants.ELEVATOR_RIGHT_MOTOR_ID, IdleMode.kBrake);

    private Rotation2d angleSetpoint = Rotation2d.kZero;

    public Elevator() {
        
    }

    @Override
    public void periodic() {
        double motorOutput = pid.calculate(getCurrentAngle().getRadians(), angleSetpoint.getRadians()) + feedforward.calculate(pid.getSetpoint().velocity);

        leftMotor.set(motorOutput);
        rightMotor.set(motorOutput);
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(leftMotor.getEncoder().getPosition());
    }

    /**
     * Set the target height of the elevator in inches
     * @param inches the height in inches
     */
    public void setHeight(double inches) {
        System.out.println("Method setHeight() is WIP. Value will not be correct.");

        final double K = 1;
        angleSetpoint = Rotation2d.fromRotations(inches * K);
        //need to convert inches to rotations but idk how to do that
    }

    /**
     * Get the target height of the elevator in inches
     * @return the current height in inches that the elevator is set to.
     */
    public double getHeightSetpoint() {
        System.out.println("Method getHeightSetpoint() is WIP. Value will not be correct.");

        final double K = 1;
        return angleSetpoint.getRotations() / K;
        //need to convert rotations to inches but idk how to do that
    }
}
