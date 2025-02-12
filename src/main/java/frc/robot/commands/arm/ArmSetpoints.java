package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmSetpoints implements ArmSetpoint {
    GROUND_CORAL(Rotation2d.fromDegrees(-20)),
    CORAL_L1(Rotation2d.fromDegrees(27)),
    CORAL_L2(Rotation2d.fromDegrees(10)),
    CORAL_L3(Rotation2d.fromDegrees(10)),
    CORAL_L4(Rotation2d.fromDegrees(20)),

    CORAL_STATION(Rotation2d.fromDegrees(75)),

    STOW(Rotation2d.fromDegrees(70)),
    HOVER_L2(Rotation2d.fromDegrees(34)),
    HOVER_L4(Rotation2d.fromDegrees(56));


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
