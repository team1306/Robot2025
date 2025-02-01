package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;
import frc.robot.util.MotorUtil;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import lombok.Getter;
import lombok.Setter;

import static edu.wpi.first.units.Units.*;


public class Elevator extends SubsystemBase {
    
    private static final double SPROCKET_DIAMETER_INCHES = 1.882;

    @GetValue
    private double elevatorP = 0, elevatorI = 0, elevatorD = 0;
    @GetValue
    private double elevatorS = 0, elevatorG = 0, elevatorV = 0; 

    private final double MAX_VELOCITY = Double.MAX_VALUE, MAX_ACCELERATION = Double.MAX_VALUE;
    private Distance PID_TOLERANCE = Inches.of(0.2);
    
    
    private final ProfiledPIDController pid;
    private final ElevatorFeedforward feedforward;

    private final TalonFXGroup motorGroup;
    private final TalonFX leftMotor, rightMotor;
    
    @Setter @Getter
    private Distance targetHeight;

    /**
     * The elevator is mounted on the robot frame and moves the arm up and down.
     * Hardware: the elevator has two Talon FX motor controllers.
     * Controllers: Feedforward and ProfiledPIDController.
     */
    public Elevator() {
        DashboardHelpers.addUpdateClass(this);
        
        leftMotor = MotorUtil.initTalonFX(Constants.ELEVATOR_LEFT_MOTOR_ID, NeutralModeValue.Brake);
        rightMotor = MotorUtil.initTalonFX(Constants.ELEVATOR_RIGHT_MOTOR_ID, NeutralModeValue.Brake, InvertedValue.CounterClockwise_Positive);
        leftMotor.setPosition(Rotations.of(0));
        rightMotor.setPosition(Rotations.of(0));
        //TODO add gear ratio into positions (in tuner x?)

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

    /**
     * Gets whether the elevator is at its setpoint using the PID controller.
     * @return true if the elevator is at its setpoint.
     */
    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    /**
     * Gets the current height of the elevator.
     * @return the elevator height in distance.
     */
    public Distance getCurrentHeight(){
        return rotationsToDistance(getCurrentElevatorMotorPositions());
    }
    
    /**
     * Gets the motor positions
     * @return the rotation of the motors
     */
    public Rotation2d getCurrentElevatorMotorPositions(){
        return Rotation2d.fromRadians(leftMotor.getPosition().getValue().plus(rightMotor.getPosition().getValue()).div(2).in(Radians));
    }
    
    /**
     * Sets the elevator motor positions to zero
     */
    public void zeroElevatorMotorPositions(){
        leftMotor.setPosition(Rotations.of(0));
        rightMotor.setPosition(Rotations.of(0));
    }

    /**
     * Converts from a Rotation2d to a Distance using the sproket diameter.
     */
    public static Distance rotationsToDistance(Rotation2d rotation) {
        return Inches.of(rotation.getRadians() * SPROCKET_DIAMETER_INCHES * Math.PI);
    }

    /**
     * Converts from a Distance to a Rotation2d using the sproket diameter.
     */
    public static Rotation2d distanceToRotations(Distance distance) {
        return Rotation2d.fromRotations(distance.in(Inches) / (SPROCKET_DIAMETER_INCHES * Math.PI));
    }
}
