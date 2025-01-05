package frc.robot.subsystems.utils;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Utilities;

@Deprecated
public class NeoGroupSubsystem extends SubsystemBase{
    public static class NeoData {
        public final SparkMax motor;
        public final boolean direction;
        public double speed; 

        public NeoData(SparkMax motor, boolean direction){
           this(motor, direction, 1);
        }

        public NeoData(SparkMax motor, boolean direction, double speed){
            this.motor = motor;
            this.direction = direction;
            this.speed = speed;
        }

        public double getSpeed(){
            return speed;
        }

        public void setSpeed(double speed){
            this.speed = speed; 
        }
    }

    public final List<NeoData> neoGroup;
    public double relativeSpeed = 0;

    @SafeVarargs
    public NeoGroupSubsystem(NeoData... neoData) {
        this.neoGroup = Utilities.arrayListFromParams(neoData);
    }

    public NeoGroupSubsystem(){
        neoGroup = new ArrayList<>();
    }

    public void addNeo(NeoData neoData){
        neoGroup.add(neoData);
    }

    @Override
    public void periodic(){
        if(neoGroup.size() < 1) throw new IllegalArgumentException("At least 1 neo needs to be registered");
        neoGroup.forEach(neo -> neo.motor.set(neo.getSpeed() * relativeSpeed));
    }
}
