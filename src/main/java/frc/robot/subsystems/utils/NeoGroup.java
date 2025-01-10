package frc.robot.subsystems.utils;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkMax;

import frc.robot.util.Utilities;

public class NeoGroup{
    public static class NeoData {
        public final SparkMax motor;
        public final boolean direction;
        public double speed; 

        public NeoData(SparkMax motor, boolean direction){
           this(motor, direction, 1);
        }

        public NeoData(SparkMax motor, boolean direction, double relativeSpeed){
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

    public final List<NeoData> neoGroup;

    @SafeVarargs
    public NeoGroup(NeoData... neoData) {
        this.neoGroup = Utilities.arrayListFromParams(neoData);
    }

    public NeoGroup(){
        neoGroup = new ArrayList<>();
    }

    public void setSpeed(double speed){
        neoGroup.forEach(neo -> neo.motor.set(neo.getSpeed() * speed));
    }
}
