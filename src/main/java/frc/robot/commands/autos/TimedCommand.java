package frc.robot.commands.autos;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class TimedCommand extends ParallelCommandGroup{
    private HashMap<Command, Double> delayMap;

    public TimedCommand(){

    }

    public final void addCommandDelay(Command command, double delay){
        delayMap.put(command, delay);
    }

    @SafeVarargs
    public final void addCommandDelays(Pair<Command, Double>... commandDelay){
        for (Pair<Command,Double> pair : commandDelay) {
            delayMap.put(pair.getFirst(), pair.getSecond());
        }
    }


}
