package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmSetpoints implements ArmSetpoint {
    GROUND_CORAL(Rotation2d.fromDegrees(-34)),
    CORAL_L1(Rotation2d.fromDegrees(-20)),
    CORAL_L2(Rotation2d.fromDegrees(25)),
    CORAL_L3(Rotation2d.fromDegrees(25)),
    CORAL_L4(Rotation2d.fromDegrees(0)),

    CORAL_STATION(Rotation2d.fromDegrees(45)),
    BUMPER_CORAL_STATION(Rotation2d.fromDegrees(75)),

    ALGAE_L2_REMOVE(Rotation2d.fromDegrees(-18)),
    ALGAE_L3_REMOVE(Rotation2d.fromDegrees(0)),

    STOW(Rotation2d.fromDegrees(65)),
    HOVER_L2(Rotation2d.fromDegrees(57)),
    HOVER_L3(Rotation2d.fromDegrees(48)),
    HOVER_L4(Rotation2d.fromDegrees(49));


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
