package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double kP = 0.15, kI = 0, kD = 0.01;
    @GetValue
    private double kG = 0.02, kV = 0; 

    //Max 5 and 1
    private final double MAX_VELOCITY = 2, MAX_ACCELERATION = 1; // placeholder
    private Distance TOLERANCE = Inches.of(0.2);
    
    private final ProfiledPIDController pid;
    private ElevatorFeedforward feedforward;

    private final TalonFXGroup motorGroup;
    private final TalonFX leftMotor, rightMotor;

    private final DigitalInput limitSwitch;

    @GetValue
    private double conversionFactor = 54.75 / 575.87;

    @GetValue
    private double maxHeightInches = 55, baseHeightInches = Constants.ELEVATOR_STARTING_HEIGHT; // placeholders
    
    @Setter @Getter
    private Distance targetHeight = Inches.of(0);

    private Distance currentHeight = Inches.of(0);

    private Distance offset = Inches.of(0);


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

        motorGroup = new TalonFXGroup(new TalonData(leftMotor), new TalonData(rightMotor));

        pid = new ProfiledPIDController(kP, kI, kD, 
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pid.setTolerance(TOLERANCE.in(Inches));

        feedforward = new ElevatorFeedforward(0, kG, kV, 0);

        limitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_ID);
        
        zeroElevatorMotorPositions();
    }

    @Override
    public void periodic() {
        pid.setPID(kP, kI, kD);
        feedforward = new ElevatorFeedforward(0, kG, kV, 0);
        if (!limitSwitch.get()) {
            offset = getRawHeight();
        }

        currentHeight = getCurrentHeight();
        SmartDashboard.putNumber("Elevator/Current Height", currentHeight.in(Inches));
        SmartDashboard.putNumber("Elevator/Target Height", targetHeight.in(Inches));

        final double target = MathUtil.clamp(targetHeight.in(Inches), baseHeightInches, maxHeightInches);
        targetHeight = Inches.of(target);

        double pidOutput = pid.calculate(currentHeight.in(Inches), target);
        double feedforwardOutput = feedforward.calculate(pid.getSetpoint().velocity);
        double motorOutput = pidOutput + feedforwardOutput;

        motorGroup.setSpeed(motorOutput);
    }

    /**
     * Gets whether the elevator is at its setpoint using the PID controller.
     * @return true if the elevator is at its setpoint.
     */
    public boolean atSetpoint() {
        return currentHeight.minus(targetHeight).abs(Inches) < TOLERANCE.in(Inches);
    }

    /**
     * Gets the current height of the elevator.
     * @return the elevator height in distance.
     */
    public Distance getCurrentHeight(){
        return getRawHeight().minus(offset);
    }

    private Distance getRawHeight() {
        return rotationsToDistance(getCurrentElevatorMotorPositions()).times(conversionFactor);
    }
    
    /**
     * Gets the motor positions
     * @return the rotation of the motors
     */
    public Rotation2d getCurrentElevatorMotorPositions(){
        return Rotation2d.fromRadians((getLeftElevatorPosition().getRadians() + getRightElevatorPosition().getRadians()) / 2D);
    }

    public Rotation2d getLeftElevatorPosition(){
        return Rotation2d.fromRadians(leftMotor.getPosition().getValue().in(Radian));
    }

    public Rotation2d getRightElevatorPosition(){
        return Rotation2d.fromRadians(rightMotor.getPosition().getValue().in(Radian));
    }
    
    /**
     * Sets the elevator motor positions to zero
     */
    public void zeroElevatorMotorPositions(){
        leftMotor.setPosition(Rotations.of(0));
        rightMotor.setPosition(Rotations.of(0));
    }

    /**
     * Converts from a Rotation2d to a Distance using the sprocket diameter.
     */
    public static Distance rotationsToDistance(Rotation2d rotation) {
        return Inches.of(rotation.getRotations() * SPROCKET_DIAMETER_INCHES * Math.PI);
    }

    /**
     * Converts from a Distance to a Rotation2d using the sprocket diameter.
     */
    public static Rotation2d distanceToRotations(Distance distance) {
        return Rotation2d.fromRotations(distance.in(Inches) / (SPROCKET_DIAMETER_INCHES * Math.PI));
    }
}
