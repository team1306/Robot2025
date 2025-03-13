package frc.robot.subsystems.utils;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;

/**
 * Fake implementation of a motor
 */
public class FakeMotor implements Motor {
    @Override
    public void set(double speed) {
        //fake implementation
    }

    @Override
    public void setPosition(Angle position) {
        //fake implementation
    }

    @Override
    public Angle getPosition() {
        return Rotations.of(0);
    }
}
