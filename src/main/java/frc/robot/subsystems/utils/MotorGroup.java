package frc.robot.subsystems.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.util.Utilities;

import java.util.List;

public class MotorGroup<MotorType extends Motor> implements Sendable {

    private final List<MotorType> motors;

    @SafeVarargs
    public MotorGroup(MotorType... motors) {
        this.motors = Utilities.arrayListFromParams(motors);
    }

    public void setSpeed(double speed) {
        motors.forEach(motor -> motor.set(speed));
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        throw new UnsupportedOperationException("Not supported yet.");
    }
}
