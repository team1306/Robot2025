package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public enum WristSetpoints implements WristSetpoint {

    HORIZONTAL(Rotation2d.fromDegrees(0)),
    VERTICAL_R(Rotation2d.fromDegrees(90)),
    VERTICAL_L(Rotation2d.fromDegrees(-90));

    private final Rotation2d rotation;

    /**
     * pre configured angles for the wrist
     *
     * @param rotation the rotation for the setpoint
     */
    WristSetpoints(Rotation2d rotation) {
        this.rotation = rotation;
    }

    @Override
    public Rotation2d getAngle() {
        return this.rotation;
    }

}
