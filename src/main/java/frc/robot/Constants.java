package frc.robot;

/**
 * Storage for robot-wide constants
 */
public final class Constants {
    private Constants(){} // block instantiation
    public static final int         INTAKE_MOTOR_ID                 = -1;
    public static final int         ARM_MOTOR_ID                    = -1;
    public static final int         WRIST_MOTOR_ID                  = -1;
    public static final int         ELEVATOR_LEFT_MOTOR_ID          = -1;
    public static final int         ELEVATOR_RIGHT_MOTOR_ID         = -1;

    public static final int         ARM_ENCODER_ID                  = 0;
    public static final int         WRIST_ENCODER_ID                = 0;
    public static final int         INTAKE_SENSOR_ID                = 0;

    public static final int         NEO_CURRENT_LIMIT_AMPS          = 60; // motor would reach safety limit after ~70s of stalling at 60A

    public static final double      LOOP_TIME_MS                    = 20;
    public static final double      LOOP_TIME_SECONDS               = LOOP_TIME_MS / 1000D;

    public static final String      LIMELIGHT_NAME                  = "limelight";
    

    public static final double      MAX_SPEED                       = 5D; // temp value, m/s
    public static final double      LEFT_X_DEADBAND                 = 0.02;
    public static final double      LEFT_Y_DEADBAND                 = 0;
    public static final double      RIGHT_X_DEADBAND                = 0;

    public static final double      ROBOT_MASS                      = 31.2; // kg

    public static final double      ELEVATOR_STARTING_HEIGHT        = 0; 
}
