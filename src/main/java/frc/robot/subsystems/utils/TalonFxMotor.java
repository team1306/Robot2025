package frc.robot.subsystems.utils;

import com.ctre.phoenix6.hardware.TalonFX;

public record TalonFxMotor(TalonFX motor, double relativeSpeed) implements MotorData{
    public TalonFxMotor(TalonFX motor){
        this(motor, 1.0);
    }
    
    @Override
    public void set(double speed) {
        motor.set(speed * relativeSpeed);
    }
}
