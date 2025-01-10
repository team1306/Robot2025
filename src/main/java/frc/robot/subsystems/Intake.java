package frc.robot.subsystems;


import frc.robot.subsystems.utils.NeoGroup;
import frc.robot.util.MotorUtil;
import lombok.Getter;
import lombok.Setter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;


public class Intake extends SubsystemBase{

    private final SparkMax motor = MotorUtil.initSparkMax(INTAKE_ID, MotorType.kBrushless, IdleMode.kBrake);
    private final NeoGroup neoGroup = new NeoGroup(new NeoGroup.NeoData(motor, false));
    private final RelativeEncoder encoder = motor.getEncoder();
    
    @Setter @Getter
    private double targetSpeed;

    @Override
    public void periodic(){
        neoGroup.setSpeed(targetSpeed);

    }
    public double getRPM() {
        return encoder.getVelocity();
    }

}