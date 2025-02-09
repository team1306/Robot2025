package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmSetpoints implements ArmSetpoint {
    GROUND_CORAL(Rotation2d.fromDegrees(-20)),
    CORAL_L1(Rotation2d.fromDegrees(0)),
    CORAL_L2(Rotation2d.fromDegrees(0)),
    CORAL_L3(Rotation2d.fromDegrees(0)),
    CORAL_L4(Rotation2d.fromDegrees(0)),

    CORAL_STATION(Rotation2d.fromDegrees(55)),

    STOW(Rotation2d.fromDegrees(70)),
    HOVER_L1(Rotation2d.fromDegrees(0)),
    HOVER_L2(Rotation2d.fromDegrees(60)),
    HOVER_L4(Rotation2d.fromDegrees(50));


    private final Rotation2d rotation;

    /**
     * pre configured setpoints for the arm
     * @param rotation the rotation for the setpoint
     */
    ArmSetpoints(Rotation2d rotation) {
        this.rotation = rotation;
    }

    @Override
    public Rotation2d getAngle() {
        return rotation;
    }
}
