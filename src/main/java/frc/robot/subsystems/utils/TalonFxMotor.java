package frc.robot.subsystems.utils;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;

public final class TalonFxMotor implements Motor {
    private final TalonFX motor;

    public TalonFxMotor(TalonFX motor) {
        this.motor = motor;
    }

    @Override
    public void set(double speed) {
        motor.set(speed);
    }

    @Override
    public void setPosition(Angle position) {
        motor.setPosition(position);
    }

    @Override
    public Angle getPosition() {
        return motor.getPosition().getValue();
    }

}
