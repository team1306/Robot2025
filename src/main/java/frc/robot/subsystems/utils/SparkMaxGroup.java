package frc.robot.subsystems.utils;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.Utilities;
import lombok.Getter;
import lombok.Setter;

public class SparkMaxGroup{
    public static class SparkMaxData {
        public final SparkMax motor;
        @Getter @Setter
        private double speed; 
    
        public SparkMaxData(SparkMax motor){
           this(motor, 1);
        }

        public SparkMaxData(SparkMax motor, double relativeSpeed){
            this.motor = motor;
            this.speed = relativeSpeed;
        }
    }

    public final List<SparkMaxData> sparkMaxGroup;

    @SafeVarargs
    public SparkMaxGroup(SparkMaxData... motorData) {
        this.sparkMaxGroup = Utilities.arrayListFromParams(motorData);
    }

    public SparkMaxGroup(){
        sparkMaxGroup = new ArrayList<>();
    }

    public void setSpeed(double speed){
        sparkMaxGroup.forEach(motorData -> motorData.motor.set(motorData.getSpeed() * speed));
    }
}
