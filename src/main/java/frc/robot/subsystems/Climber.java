package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import frc.robot.util.Dashboard.PutValue;
import frc.robot.util.MotorUtil;
import lombok.Getter;
import lombok.Setter;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.*;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;

public class Climber extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFXGroup motorGroup;
    
    @GetValue
    private boolean enforceMaxPosition = false, enforceMinPosition = false;

    @GetValue
    private double minPosition = -100, maxPosition = 100; // placeholder, pos = further into robot

    @GetValue
    private static double MAX_SPEED = 1;

    @Getter @Setter
    private double targetSpeed;

    @PutValue
    private double motorPosition;

    @Getter @Setter @PutValue
    private double motorSetpoint;

    @GetValue
    private double kP, kI, kD;

    private PIDController pidController;

    public Climber() {
        DashboardHelpers.addUpdateClass(this);
        
        motor = MotorUtil.initTalonFX(CLIMB_MOTOR_ID, NeutralModeValue.Brake);
        motorGroup = new TalonFXGroup(new TalonData(motor));

        // Following previous style guidelines, all of these should be configured in TunerX.
        // This may be changed for the future, but for now we should stay consistent
        // TODO should double check that creating the motor in code doesn't reset config
//        final CurrentLimitsConfigs currentsConfig = new CurrentLimitsConfigs();
//        currentsConfig.StatorCurrentLimit = 80;
//        currentsConfig.StatorCurrentLimitEnable = true;
//        currentsConfig.SupplyCurrentLimitEnable = false;
//
//        final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
//        feedbackConfigs.SensorToMechanismRatio = RATIO;
//
//        TalonFXConfigurator configurator = motor.getConfigurator();
//        configurator.apply(feedbackConfigs);
//        configurator.apply(currentsConfig);

        //Todo if the climber starts down, the motor position should definitely not be 0
        //Todo otherwise the min and max positions need to be readjusted
        pidController = new PIDController(kP, kI, kD);
        motor.setPosition(0);
    }
    
    public boolean isPastMax() {
        return enforceMaxPosition && motorPosition > maxPosition;
    }

    public boolean isPastMin() {
        return enforceMinPosition && motorPosition < minPosition;
    }

    @Override
    public void periodic() {
        motorPosition = motor.getPosition().getValue().in(Rotations);
        pidController.setPID(kP, kI, kD);
        double output = pidController.calculate(motorPosition, motorSetpoint);
        
        // double motorSpeed = 
        //     switch ((int) Math.signum(speed)) {
        //         case -1 -> isPastMin() ? 0 : speed;
        //         case 1 -> isPastMax() ? 0 : speed;
        //         default -> 0;
        // };
        
        motorGroup.setSpeed(targetSpeed * MAX_SPEED);
    }
}
