package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

@FunctionalInterface
public interface WristSetpoint {
    Rotation2d getAngle();
}
