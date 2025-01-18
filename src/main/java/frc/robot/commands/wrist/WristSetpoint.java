package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public enum WristSetpoint {

    HORAZONTAL(Rotation2d.fromDegrees(90)),
    VERTICAL(Rotation2d.fromDegrees(0));

    public final Rotation2d rotation;

    private WristSetpoint(Rotation2d rotation) {
        this.rotation = rotation;
    }

}
