package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class CustomWaitCommand extends Command{
    private final DoubleSupplier duration;
    protected Timer m_timer = new Timer();

    public CustomWaitCommand(DoubleSupplier timeSupplier){
        this.duration = timeSupplier;
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(duration.getAsDouble());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

}
