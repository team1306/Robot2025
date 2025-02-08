package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Sets an LEDStrip objects color
 * @param subsystem LEDSubsystem object
 * @param color takes a Color8Bit object
 */
public class FillLEDColor extends Command{
    public static Command fillColor(LEDSubsystem subsystem, Color8Bit color){
        return Commands.runOnce(() -> subsystem.fillAndCommitColor(color), subsystem);
    }
    public static Command flashColor(LEDSubsystem subsystem, Color8Bit color){
        return Commands.repeatingSequence(
            Commands.runOnce(() -> subsystem.fillAndCommitColor(color), subsystem),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> subsystem.fillAndCommitColor(Constants.LED_OFF), subsystem),
            Commands.waitSeconds(1)
        );
    }
}
