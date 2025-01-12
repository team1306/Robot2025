package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmSetpoints {

    EXAMPLE(Rotation2d.fromDegrees(90));

    public final Rotation2d rotation;

    private ArmSetpoints(Rotation2d rotation) {
        this.rotation = rotation;
    }
}
