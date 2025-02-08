package frc.robot.commands.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LEDSubsystem;

public class LEDPatterns extends Command {
    public static Command setRainbowEffect(LEDSubsystem subsystem){
        final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
        final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(LinearVelocity.ofBaseUnits(0.1,MetersPerSecond), Distance.ofBaseUnits((1/120), Meters));
        return Commands.runOnce(() -> subsystem.applyPattern(m_scrollingRainbow), subsystem);
    }
}
