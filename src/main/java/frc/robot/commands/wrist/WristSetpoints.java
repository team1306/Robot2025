package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public enum WristSetpoints {

    OPEN(Rotation2d.fromDegrees(90));

    public final Rotation2d rotation;

    private WristSetpoints(Rotation2d rotation) {
        this.rotation = rotation;
    }

}
