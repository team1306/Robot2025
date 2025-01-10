package frc.robot.subsystems.utils;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.Utilities;

public class TalonGroup{
    public static class TalonData {
        public final TalonFX motor;
        public final boolean direction;
        public double speed; 

        public TalonData(TalonFX motor, boolean direction){
           this(motor, direction, 1);
        }

        public TalonData(TalonFX motor, boolean direction, double relativeSpeed){
            this.motor = motor;
            this.direction = direction;
            this.speed = relativeSpeed;
        }

        public double getSpeed(){
            return speed;
        }

        public void setSpeed(double speed){
            this.speed = speed; 
        }
    }

    public final List<TalonData> TalonGroup;

    @SafeVarargs
    public TalonGroup(TalonData... TalonData) {
        this.TalonGroup = Utilities.arrayListFromParams(TalonData);
    }

    public TalonGroup(){
        TalonGroup = new ArrayList<>();
    }

    public void setSpeed(double speed){
        TalonGroup.forEach(Talon -> Talon.motor.set(Talon.getSpeed() * speed));
    }
}
