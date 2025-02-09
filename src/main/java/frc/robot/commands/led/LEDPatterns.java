package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Inches;
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
        final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
        final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.1,MetersPerSecond), Distance.ofBaseUnits((1/120), Meters));
        
        return Commands.runOnce(() -> subsystem.applyPattern(m_scrollingRainbow), subsystem);
    }

    public static Command elevatorHeightRainbowMask(LEDSubsystem subsystem, Elevator elevator){
        final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
        final LEDPattern base = m_rainbow.scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.1,MetersPerSecond), Distance.ofBaseUnits((1/120), Meters));

        DoubleSupplier doubleSupplier = () -> {
            Distance height = elevator.getCurrentHeight();
            Distance maxHeight = Distance.ofBaseUnits(55, Inches);
            return height.in(Inches)/maxHeight.in(Inches); 
        };

        final LEDPattern mask = LEDPattern.progressMaskLayer(doubleSupplier);
        final LEDPattern heightDisplay = base.mask(mask);

        return Commands.runOnce(() -> subsystem.applyPattern(heightDisplay), subsystem);
    }
}
