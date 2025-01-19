package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmSetpoint {

    CORAL_L1(Rotation2d.fromDegrees(90)),
    CORAL_L2(Rotation2d.fromDegrees(35)),
    CORAL_L3(Rotation2d.fromDegrees(35)),
    CORAL_L4(Rotation2d.fromDegrees(90)),

    CORAL_STATION(Rotation2d.fromDegrees(55));

    public final Rotation2d rotation;

    private ArmSetpoint(Rotation2d rotation) {
        this.rotation = rotation;
    }
}
