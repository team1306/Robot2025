package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.utils.SparkMaxGroup;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;
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
    private Distance PID_TOLERANCE = Inches.of(0.2);
    private ProfiledPIDController pid;

    private ElevatorFeedforward feedforward;

    private final TalonFXGroup motorGroup;
    private final TalonFX leftMotor, rightMotor;

    private final DutyCycleEncoder elevatorEncoder;

    @Setter @Getter
    private Distance targetHeight;

    public Elevator() {
        DashboardHelpers.addUpdateClass(this);
        
        leftMotor = MotorUtil.initTalonFX(Constants.ELEVATOR_LEFT_MOTOR_ID, NeutralModeValue.Brake);
        rightMotor = MotorUtil.initTalonFX(Constants.ELEVATOR_RIGHT_MOTOR_ID, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);

        elevatorEncoder = new DutyCycleEncoder(0);

        motorGroup = new TalonFXGroup(new TalonData(leftMotor), new TalonData(rightMotor));

        pid = new ProfiledPIDController(elevatorP, elevatorI, elevatorD, 
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pid.setTolerance(PID_TOLERANCE.in(Inches));

        feedforward = new ElevatorFeedforward(elevatorS, elevatorG, elevatorV);
    }

    @Override
    public void periodic() {
        double pidOutput = pid.calculate(getCurrentHeight().in(Inches), targetHeight.in(Inches));
        double feedforwardOutput = feedforward.calculate(pid.getSetpoint().velocity);
        double motorOutput = pidOutput + feedforwardOutput;

        motorGroup.setSpeed(motorOutput);
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public Distance getCurrentHeight(){
        return rotationsToDistance(elevatorEncoder.get());
    }

    public static Distance rotationsToDistance(double rotations) {
        return Inches.of(rotations * SPROCKET_DIAMETER_INCHES * Math.PI);
    }
    public static double distanceToRotations(Distance distance) {
        return distance.in(Inches) / (SPROCKET_DIAMETER_INCHES * Math.PI);
    }
}
