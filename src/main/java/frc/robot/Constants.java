package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * Storage for robot-wide constants
 */
public final class Constants {
    private Constants(){} // block instantiation

    public static final int WRIST_MOTOR_ID = -1;

    public static final int           NEO_COUNTS_PER_REVOLUTION      = 42;
    public static final int           NEO_CURRENT_LIMIT_AMPS         = 60; // motor would reach safety limit after ~70s of stalling at 60A
    public static final double        NEO_MAX_VOLTAGE                = 12;
    //IDs 1 - 8 are taken by swerve (at the moment)
    public static final double        LOOP_TIME_MS                   = 20;
    public static final double        LOOP_TIME_SECONDS              = LOOP_TIME_MS / 1000D;


    public static final String        LIMELIGHT_NAME                 = "limelight";
        
    public static final boolean       INCLUDE_AUTO                   = true;
    public static final boolean       INCLUDE_LIMELIGHT              = true;

    public static final double        MAX_SPEED                      = 5D; // temp value, m/s
    public static final double        LEFT_X_DEADBAND                = 0;
    public static final double        LEFT_Y_DEADBAND                = 0;
    public static final double        RIGHT_X_DEADBAND               = 0;

    public static final double ROBOT_MASS = 31.2; // kg
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    
}
