package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmSetpoint {

    GROUND_CORAL(Rotation2d.fromDegrees(-10)),
    CORAL_L1(Rotation2d.fromDegrees(0)),
    CORAL_L2(Rotation2d.fromDegrees(0)),
    CORAL_L3(Rotation2d.fromDegrees(0)),
    CORAL_L4(Rotation2d.fromDegrees(0)),

    CORAL_STATION(Rotation2d.fromDegrees(55)),

    STOW(Rotation2d.fromDegrees(80)),
    HOVER(Rotation2d.fromDegrees(60));

    public final Rotation2d rotation;

    /**
     * pre configured setpoints for the arm
     * @param rotation the rotation for the setpoint
     */
    ArmSetpoint(Rotation2d rotation) {
        this.rotation = rotation;
    }
}
