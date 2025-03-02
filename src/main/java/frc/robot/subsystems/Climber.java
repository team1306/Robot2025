package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.TalonFXGroup;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import frc.robot.util.Dashboard.PutValue;
import frc.robot.util.MotorUtil;
import lombok.Getter;
import lombok.Setter;
import lombok.SneakyThrows;

import static frc.robot.Constants.*;
import frc.robot.subsystems.utils.TalonFXGroup.TalonData;

public class Climber extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFXGroup motorGroup;
    private final LaserCan laser;
    
    private static final double SENSOR_PIVOT_DISTANCE_MM = 80.9879;

    @GetValue
    private boolean enforceMaxPosition = true, enforceMinPosition = true;

    @GetValue
    private double minPosition = 2.5, maxPosition = 26.25; // placeholder, pos = further into robot

    @GetValue
    private static double MAX_SPEED = 1;

    @Getter @Setter
    private double targetSpeed;

    @PutValue
    private double distance = -1D;

    @Getter @Setter @PutValue
    private double distanceSetpoint;

    @SneakyThrows(ConfigurationFailedException.class)
    public Climber() {
        DashboardHelpers.addUpdateClass(this);
        
        motor = MotorUtil.initTalonFX(CLIMB_MOTOR_ID, NeutralModeValue.Brake);

        motorGroup = new TalonFXGroup(new TalonData(motor));
        laser = new LaserCan(CLIMBER_LASER_CAN_ID);
        laser.setRangingMode(LaserCan.RangingMode.SHORT);
        laser.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);

    }
    
    public boolean isPastMax() {
        return enforceMaxPosition && distance > maxPosition;
    }

    public boolean isPastMin() {
        return enforceMinPosition && distance < minPosition;
    }

    @Override
    public void periodic() {
        final LaserCan.Measurement measurement = laser.getMeasurement();
        distance = measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm != 0D 
                ? 90 - Math.toDegrees(Math.atan(SENSOR_PIVOT_DISTANCE_MM/measurement.distance_mm))
                : distance;
        motorGroup.setSpeed(
            switch ((int) Math.signum(targetSpeed)) {
                case -1 -> isPastMax() ? 0 : targetSpeed;
                case 1 -> isPastMin() ? 0 : targetSpeed;
                default -> 0;
            } * MAX_SPEED
        );
    }
}
