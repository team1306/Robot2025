package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import lombok.Getter;
import lombok.Setter;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.*;


public class Climber extends SubsystemBase {
    private final TalonFX motor;
    private final static double RATIO = 20D;

    @GetValue
    private boolean enforceMaxPosition = false, enforceMinPosition = false;

    @GetValue
    private double minPosition = -100, maxPosition = 100; // placeholder, pos = further into robot

    @GetValue
    private static double MAX_SPEED;

    @Getter
    @Setter
    private double speed;

    private double motorPosition;

    public Climber() {
        DashboardHelpers.addUpdateClass(this);
        motor = new TalonFX(CLIMB_MOTOR_ID);

        final CurrentLimitsConfigs currentsConfig = new CurrentLimitsConfigs();
        currentsConfig.StatorCurrentLimit = 80;
        currentsConfig.StatorCurrentLimitEnable = true;
        currentsConfig.SupplyCurrentLimitEnable = false;

        final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = RATIO;

        TalonFXConfigurator configurator = motor.getConfigurator();
        configurator.apply(feedbackConfigs);
        configurator.apply(currentsConfig);

        motor.setPosition(0);
    }
    
    public boolean isPastMax() {
        return enforceMaxPosition && motorPosition > maxPosition;
    }

    public boolean isPastMin() {
        return enforceMinPosition && motorPosition < minPosition;
    }

    public void periodic() {
        motorPosition = motor.getPosition().getValue().in(Rotations);
        motor.set(switch ((int) Math.signum(speed)) {
            case -1 -> (!isPastMin()) ? speed : 0;
            case 1 -> (!isPastMax()) ? speed : 0;
            default -> 0;
        });
    }
}
