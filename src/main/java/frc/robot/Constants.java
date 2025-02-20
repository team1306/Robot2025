package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Storage for robot-wide constants
 */
public final class Constants {
    private Constants(){} // block instantiation
    public static final int         INTAKE_MOTOR_ID                 = 10;
    public static final int         ARM_MOTOR_ID                    = 30;
    public static final int         WRIST_MOTOR_ID                  = 20;
    public static final int         ELEVATOR_LEFT_MOTOR_ID          = 40;
    public static final int         ELEVATOR_RIGHT_MOTOR_ID         = 41;
    public static final int         CLIMB_MOTOR_ID                  = 50;

    public static final int         ARM_ENCODER_ID                  = 2;
    public static final int         WRIST_ENCODER_ID                = 1;
    public static final int         INTAKE_SENSOR_ID                = 0;
    public static final int         ELEVATOR_LIMIT_SWITCH_ID        = 3;

    public static final int         NEO_CURRENT_LIMIT_AMPS          = 80; // motor would reach safety limit after ~70s of stalling at 60A

    public static final double      LOOP_TIME_MS                    = 20;
    public static final double      LOOP_TIME_SECONDS               = LOOP_TIME_MS / 1000D;

    public static final String      LIMELIGHT_NAME                  = "limelight";
    
    public static final double      MAX_SPEED                       = 5D; // temp value, m/s
    public static final double      LEFT_X_DEADBAND                 = 0.02;
    public static final double      LEFT_Y_DEADBAND                 = 0;
    public static final double      RIGHT_X_DEADBAND                = 0;

    public static final double      ROBOT_MASS                      = Units.lbsToKilograms(130); // kg

    public static final double      ELEVATOR_STARTING_HEIGHT        = 0;

    public static final int LED_PORT = 1; // pwm port
    public static final int LED_COUNT = 50; // pixels
    public static final Color8Bit RED = new Color8Bit(255,0,0);
    public static final Color8Bit BLUE = new Color8Bit(0,0,255);
    public static final Color8Bit LED_OFF = new Color8Bit(0,0,0);

    public enum Direction {
        FORWARD,
        REVERSE
    }
}
