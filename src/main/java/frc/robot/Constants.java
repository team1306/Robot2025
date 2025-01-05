package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Storage for robot-wide constants
 */
public final class Constants {
    private Constants(){} // block instantiation

    public static final int           NEO_COUNTS_PER_REVOLUTION      = 42;
    public static final int           NEO_CURRENT_LIMIT_AMPS         = 60; // motor would reach safety limit after ~70s of stalling at 60A
    public static final double        NEO_MAX_VOLTAGE                = 12;
 
    public static final int           FRONT_LEFT_DRIVE_MOTOR_ID      = 1;
    public static final int           FRONT_RIGHT_DRIVE_MOTOR_ID     = 2;
    public static final int           BACK_LEFT_DRIVE_MOTOR_ID       = 3;
    public static final int           BACK_RIGHT_DRIVE_MOTOR_ID      = 4;
    public static final int           FRONT_LEFT_DRIVE_STEER_ID      = 1;
    public static final int           FRONT_RIGHT_DRIVE_STEER_ID     = 2;
    public static final int           BACK_LEFT_DRIVE_STEER_ID       = 3;
    public static final int           BACK_RIGHT_DRIVE_STEER_ID      = 4;
 
    public static final int           INTAKE_FRONT_MOTOR_ID          = 5;
    public static final int           INTAKE_LEFT_MOTOR_ID           = 6;
    public static final int           INTAKE_RIGHT_MOTOR_ID          = 7;
    public static final int           INTAKE_CENTER_OMNI_MOTOR_ID    = 8;
    public static final int           INTAKE_CENTER_BELT_MOTOR_ID    = 9;

    public static final int           INDEXER_MOTOR_ID               = 10; 

    public static final int           SHOOTER_TOP_MOTOR_ID           = 11;
    public static final int           SHOOTER_BOTTOM_MOTOR_ID        = 12;
 
    public static final int           PIVOTER_LEFT_MOTOR_ID          = 13;
    public static final int           PIVOTER_RIGHT_MOTOR_ID         = 14;

    public static final int           TURRET_MOTOR_ID                = 15;


    public static final int           INTAKE_SENSOR_DIO_PORT         = 0;
    public static final int           INDEXER_SENSOR_DIO_PORT        = 1;

    public static final int           TURRET_ENCODER_DIO_PORT_A      = 2;
    public static final int           TURRET_ENCODER_DIO_PORT_B      = 3;

    public static final int           PIVOTER_LEFT_ENCODER_DIO_PORT  = 4;
    public static final int           PIVOTER_RIGHT_ENCODER_DIO_PORT = 5;

 
    public static final double        LOOP_TIME_MS                   = 20;
    public static final double        LOOP_TIME_SECONDS              = LOOP_TIME_MS / 1000D;


    public static final String        LIMELIGHT_NAME                 = "limelight";
    public static final Translation2d BLUE_SPEAKER                   = new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));
    public static final Translation2d RED_SPEAKER                    = new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));
           
    public static final boolean       INCLUDE_AUTO                   = true;
    public static final boolean       INCLUDE_LIMELIGHT              = true;

    public static final double        MAX_SPEED                      = 5D; // temp value, m/s
    public static final double        LEFT_X_DEADBAND                = 0;
    public static final double        LEFT_Y_DEADBAND                = 0D;
    public static final double        RIGHT_X_DEADBAND               = 0;
}
