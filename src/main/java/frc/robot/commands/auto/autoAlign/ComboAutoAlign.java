package frc.robot.commands.auto.autoAlign;

import edu.wpi.first.wpilibj2.command.Command;

public class ComboAutoAlign extends Command {

    private final Runnable globalAutoAlign;
    private final Runnable localAutoAlign;

    private Runnable currentAutoAlign;

    public ComboAutoAlign(Runnable globalAutoAlign, Runnable localAutoAlign) {
        this.globalAutoAlign = globalAutoAlign;
        this.localAutoAlign = localAutoAlign;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (true) { //TODO: change to use LL info
            currentAutoAlign = globalAutoAlign;
        } else {
            currentAutoAlign = localAutoAlign;
        }

        currentAutoAlign.run();
    }

    @Override
    public boolean isFinished() {return true;}

    @Override
    public void end(boolean interrupted) {
    }
}
