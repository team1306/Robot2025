package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.utils.SparkMaxGroup;
import frc.robot.subsystems.utils.SparkMaxGroup.SparkMaxData;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import lombok.Getter;
import lombok.Setter;


public class Elevator extends SubsystemBase {
    
    // go to and hold positions (thrifty elevator) [PID and Feedforward]
    private static final double SPROCKET_DIAMETER_INCHES = 1.882;

    @GetValue
    private double elevatorP = 0, elevatorI = 0, elevatorD = 0;
    @GetValue
    private double elevatorS = 0, elevatorG = 0, elevatorV = 0; 

    private final double MAX_VELOCITY = Double.MAX_VALUE, MAX_ACCELERATION = Double.MAX_VALUE;
    private ProfiledPIDController pid = new ProfiledPIDController(elevatorP, elevatorI, elevatorD, 
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(elevatorS, elevatorG, elevatorV);

    private final SparkMaxGroup motorGroup;
    private final SparkMax leftMotor, rightMotor;

    private final DutyCycleEncoder elevatorEncoder;

    @Setter @Getter
    private Distance elevatorSetpoint;

    public Elevator() {
        DashboardHelpers.addUpdateClass(this);
        
        leftMotor = MotorUtil.initSparkMax(Constants.ELEVATOR_LEFT_MOTOR_ID, IdleMode.kBrake);
        rightMotor = MotorUtil.initSparkMax(Constants.ELEVATOR_RIGHT_MOTOR_ID, IdleMode.kBrake, true);

        elevatorEncoder = new DutyCycleEncoder(0);

        motorGroup = new SparkMaxGroup(new SparkMaxData(leftMotor), new SparkMaxData(rightMotor));
    }

    @Override
    public void periodic() {
        double pidOutput = pid.calculate(getCurrentHeight().in(Inches), elevatorSetpoint.in(Inches));
        double feedforwardOutput = feedforward.calculate(pid.getSetpoint().velocity);
        double motorOutput = pidOutput + feedforwardOutput;

        motorGroup.setSpeed(motorOutput);
    }

    public Distance getCurrentHeight(){
        return rotationsToDistance(elevatorEncoder.get());
    }

    public static Distance rotationsToDistance(double rotations) {
        return Distance.ofBaseUnits(rotations * SPROCKET_DIAMETER_INCHES * Math.PI, Inches);
    }
    public static double inchesToRotations(double inches) {
        return inches / (SPROCKET_DIAMETER_INCHES * Math.PI);
    }
}
