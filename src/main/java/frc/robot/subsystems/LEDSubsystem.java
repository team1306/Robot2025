package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDSegment;

public class LEDSubsystem extends SubsystemBase {
    protected final LEDSegment leds;

    public LEDSubsystem(int firstLED, int ledCount) {
        leds = new LEDSegment(firstLED, ledCount);
    }

    public void setColor(Color8Bit color, int index) {
        leds.setColor(color, index);
    }

    public void fillAndCommitColor(Color8Bit color) {
        leds.fill(color);
        commitColor();
    }

    public void commitColor() {
        leds.commitColor();
    }

    public void fillColor(Color8Bit color) {
        leds.fill(color);
    }
}