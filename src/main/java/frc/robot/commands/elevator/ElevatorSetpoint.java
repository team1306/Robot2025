package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Distance;

@FunctionalInterface
public interface ElevatorSetpoint {
    Distance getHeight();
}
