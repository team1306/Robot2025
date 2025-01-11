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

    private static final double SPROCKET_DIAMETER_INCHES = 1.882;

    private final double PID_P = 0, PID_I = 0, PID_D = 0, //PID constants
            MAX_VELOCITY = Double.MAX_VALUE, MAX_ACCELERATION = Double.MAX_VALUE;
    private final double FF_S = 0, FF_G = 0, FF_V = 0; //FF constants

    private ProfiledPIDController pid = new ProfiledPIDController(PID_P, PID_I, PID_D, 
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
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

    public void setHeight(double inches) {
        angleSetpoint = Rotation2d.fromRotations(inchesToRotations(inches));
    }

    public double getHeightSetpoint() {
        return rotationsToInches(angleSetpoint.getRotations());
        //need to convert rotations to inches but idk how to do that
    }

    public static double rotationsToInches(double rotations) {
        return rotations * SPROCKET_DIAMETER_INCHES * Math.PI;
    }
    public static double inchesToRotations(double inches) {
        return inches / (SPROCKET_DIAMETER_INCHES * Math.PI);
    }
}
