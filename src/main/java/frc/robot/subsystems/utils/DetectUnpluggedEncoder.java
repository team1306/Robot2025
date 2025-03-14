package frc.robot.subsystems.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;

public class DetectUnpluggedEncoder {

    private final double timeThresholdSeconds = 2.0;

    private DoubleSupplier encoderSupplier;
    private DoubleSupplier targetSupplier;
    private Timer timer = new Timer();

    public DetectUnpluggedEncoder (DoubleSupplier encoderSupplier, DoubleSupplier targetSupplier) {
        this.encoderSupplier = encoderSupplier;
        this.targetSupplier = targetSupplier;
        timer.restart();
    }

    /**
     * Run each loop to detect if encoder is unplugged
     * @return true if encoder is unplugged
     */
    public boolean update() {
        if (targetSupplier.getAsDouble() == 0 || encoderSupplier.getAsDouble() != 0) {
            timer.restart();
            return false;
        }
        
        if (timer.get() > timeThresholdSeconds) {
            return true;
        }

        return false;
    }    
}
