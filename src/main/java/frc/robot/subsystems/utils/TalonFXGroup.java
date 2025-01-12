package frc.robot.subsystems.utils;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.Utilities;
import lombok.Getter;
import lombok.Setter;

public class TalonFXGroup{
    public static class TalonData {
        public final TalonFX motor;
        public final boolean inverted;
        @Getter @Setter
        private double speed; 

        public TalonData(TalonFX motor, boolean inverted){
           this(motor, inverted, 1);
        }

        public TalonData(TalonFX motor, boolean inverted, double relativeSpeed){
            this.motor = motor;
            this.inverted = inverted;
            this.speed = relativeSpeed;
        }
    }

    public final List<TalonData> talonFXGroup;

    @SafeVarargs
    public TalonFXGroup(TalonData... TalonData) {
        talonFXGroup = Utilities.arrayListFromParams(TalonData);
    }

    public TalonFXGroup(){
        talonFXGroup = new ArrayList<>();
    }

    public void setSpeed(double speed){
        talonFXGroup.forEach(motorData -> motorData.motor.set(motorData.getSpeed() * speed));
    }
}
