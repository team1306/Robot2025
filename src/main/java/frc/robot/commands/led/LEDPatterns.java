package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;

public class LEDPatterns extends Command {
    public static Command setRainbowEffect(LEDSubsystem subsystem){
        final LEDPattern rainbow = LEDPattern.rainbow(255, 75);
        final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(InchesPerSecond.of(40), Meters.of(1.0/60));
        
        return Commands.run(() -> subsystem.applyPattern(scrollingRainbow), subsystem);
    }

    public static Command elevatorHeightRainbowMask(LEDSubsystem subsystem, Elevator elevator){
        final LEDPattern rainbow = LEDPattern.rainbow(255, 75);
        final LEDPattern base = rainbow.scrollAtAbsoluteSpeed(InchesPerSecond.of(40), Meters.of(1.0/60));

        DoubleSupplier doubleSupplier = () -> {
            Distance height = elevator.getCurrentHeight();
            Distance maxHeight = Inches.of(55);
            return height.in(Inches)/maxHeight.in(Inches); 
        };

        final LEDPattern mask = LEDPattern.progressMaskLayer(doubleSupplier);
        final LEDPattern heightDisplay = base.mask(mask);

        return Commands.run(() -> subsystem.applyPattern(heightDisplay), subsystem);
    }
}
