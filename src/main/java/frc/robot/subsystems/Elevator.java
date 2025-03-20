package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.utils.Motor;
import frc.robot.subsystems.utils.MotorGroup;
import frc.robot.subsystems.utils.TalonFxMotor;
import frc.robot.util.MotorUtil;
import frc.robot.util.dashboardv3.entry.Config;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import frc.robot.util.dashboardv3.networktables.mappings.UnitMappings;
import lombok.Getter;
import lombok.Setter;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
    
    private static final double SPROCKET_DIAMETER_INCHES = 1.882;

    @Entry(type = EntryType.Subscriber)
    private static double kG = 0.045, kV = 0;

    private final static double MAX_VELOCITY = 1e+9, MAX_ACCELERATION = 700; // placeholder
    private final Distance TOLERANCE = Inches.of(0.5);
    
    @Entry(type = EntryType.Sendable)
    private static ProfiledPIDController pid = new ProfiledPIDController(0.1, 0, 0.006,  new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
    private ElevatorFeedforward feedforward;

    private final MotorGroup<Motor> motorGroup;
    private final Motor leftMotor, rightMotor;

    private final DigitalInput limitSwitch;

    @Entry(type = EntryType.Subscriber)
    private static double conversionFactor = 54.75 / 344.69 * 3;

    @Entry(type = EntryType.Subscriber)
    private static double maxHeightInches = 53.5, baseHeightInches = Constants.ELEVATOR_STARTING_HEIGHT; // placeholders
    
    @Setter @Getter
    @Entry(type = EntryType.Publisher)
    @Config(UnitMappings.DistanceConfiguration.INCHES)
    private static Distance targetHeight = Inches.of(0);

    @Entry(type = EntryType.Publisher)
    @Config(UnitMappings.DistanceConfiguration.INCHES)
    private static Distance currentHeight = Inches.of(0);

    private Distance offset = Inches.of(0);

    /**
     * The elevator is mounted on the robot frame and moves the arm up and down.
     * Hardware: the elevator has two Talon FX motor controllers.
     * Controllers: Feedforward and ProfiledPIDController.
     */
    public Elevator() {
       leftMotor = new TalonFxMotor(MotorUtil.initTalonFX(Constants.ELEVATOR_LEFT_MOTOR_ID, NeutralModeValue.Brake));
       rightMotor = new TalonFxMotor(MotorUtil.initTalonFX(Constants.ELEVATOR_RIGHT_MOTOR_ID, NeutralModeValue.Brake));
        // leftMotor = new FakeMotor();
        // rightMotor = new FakeMotor();

        leftMotor.setPosition(Rotations.of(0));
        rightMotor.setPosition(Rotations.of(0));

        motorGroup = new MotorGroup<>(leftMotor, rightMotor);

        pid.setTolerance(TOLERANCE.in(Inches));

        feedforward = new ElevatorFeedforward(0, kG, kV, 0);

        limitSwitch = new DigitalInput(Constants.ELEVATOR_LIMIT_SWITCH_ID);
        zeroElevatorMotorPositions();
    }

    @Override
    public void periodic() {
        feedforward = new ElevatorFeedforward(0, kG, kV, 0);
        if (!limitSwitch.get()) {
            offset = getRawHeight();
        }

        currentHeight = getCurrentHeight();

        final double target = MathUtil.clamp(targetHeight.in(Inches), baseHeightInches, maxHeightInches);
        targetHeight = Inches.of(target);

        double pidOutput = pid.calculate(currentHeight.in(Inches), target);
        double feedforwardOutput = feedforward.calculate(pid.getSetpoint().velocity);
        double motorOutput = pidOutput + feedforwardOutput;

        motorGroup.setSpeed(motorOutput);
    }
    
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
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
        return Rotation2d.fromRadians(leftMotor.getPosition().in(Radian));
    }

    public Rotation2d getRightElevatorPosition(){
        return Rotation2d.fromRadians(rightMotor.getPosition().in(Radian));
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
