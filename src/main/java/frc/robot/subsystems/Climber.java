package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.utils.FakeMotor;
import frc.robot.subsystems.utils.Motor;
import frc.robot.subsystems.utils.MotorGroup;
import frc.robot.subsystems.utils.TalonFxMotor;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;
import frc.robot.util.MotorUtil;
import lombok.Getter;
import lombok.Setter;
import lombok.SneakyThrows;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {
    private final MotorGroup<Motor> motorGroup;
    private LaserCan laser = null;
    
    private static final double SENSOR_PIVOT_DISTANCE_MM = 80.9879;

    @Entry(type = EntryType.Subscriber)
    private static boolean enforceMaxPosition = true, enforceMinPosition = true;

    @Entry(type = EntryType.Subscriber)
    private static double minPosition = 2.5, maxPosition = 26.25; // placeholder, pos = further into robot

    @Entry(type = EntryType.Subscriber)
    private static double MAX_SPEED = 1;

    @Getter @Setter
    private double targetSpeed;

    @Entry(type = EntryType.Publisher)
    private static double distance = -1D;

    @Getter @Setter @Entry(type = EntryType.Publisher)
    private static double distanceSetpoint;

    @SneakyThrows(ConfigurationFailedException.class)
    public Climber() {
//        Motor motor = new TalonFxMotor(MotorUtil.initTalonFX(CLIMB_MOTOR_ID, NeutralModeValue.Brake));
        Motor motor = new FakeMotor();

        motorGroup = new MotorGroup<>(motor);
        //TODO remove to enable laserCAN
        if(RobotBase.isReal() && false){
            laser = new LaserCan(CLIMBER_LASER_CAN_ID);
            laser.setRangingMode(LaserCan.RangingMode.SHORT);
            laser.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
        }

    }
    
    public boolean isPastMax() {
        return enforceMaxPosition && distance > maxPosition;
    }

    public boolean isPastMin() {
        return enforceMinPosition && distance < minPosition;
    }

    @Override
    public void periodic() {
        final LaserCan.Measurement measurement = laser != null ? laser.getMeasurement() : null;
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
