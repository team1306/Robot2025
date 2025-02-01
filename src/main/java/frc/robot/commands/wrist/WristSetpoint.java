package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public enum WristSetpoint {

    HORIZONTAL(Rotation2d.fromDegrees(90)),
    VERTICAL(Rotation2d.fromDegrees(0));

    public final Rotation2d rotation;

    /**
     * pre configured angles for the wrist
     * @param rotation the rotation for the setpoint
     */
    WristSetpoint(Rotation2d rotation) {
        this.rotation = rotation;
    }

}
