package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class FillLEDColor extends Command{
    public static Command fillColor(LEDSubsystem subsystem, Color8Bit color){
        return Commands.runOnce(() -> subsystem.fillAndCommitColor(color), subsystem);
    }
    public static Command flashColor(LEDSubsystem subsystem, Color8Bit color, int seconds){
        return Commands.repeatingSequence(
            Commands.runOnce(() -> subsystem.fillAndCommitColor(color), subsystem),
            Commands.waitSeconds(seconds),
            Commands.runOnce(() -> subsystem.fillAndCommitColor(Constants.LED_OFF), subsystem),
            Commands.waitSeconds(seconds)
        );
    }
    public static Command flashTwoColors(LEDSubsystem subsystem, Color8Bit colorOne, Color8Bit colorTwo, int seconds){
        return Commands.repeatingSequence(
            Commands.runOnce(() -> subsystem.fillAndCommitColor(colorOne), subsystem),
            Commands.waitSeconds(seconds),
            Commands.runOnce(() -> subsystem.fillAndCommitColor(colorTwo), subsystem),
            Commands.waitSeconds(seconds)
        );
    }
    public static Command turnOff(LEDSubsystem subsystem){
        return Commands.runOnce(() -> subsystem.fillAndCommitColor(Constants.LED_OFF), subsystem);
    }
}