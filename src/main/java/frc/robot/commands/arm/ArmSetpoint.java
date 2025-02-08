package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;

@FunctionalInterface
public interface ArmSetpoint {
    Rotation2d getAngle();
}
