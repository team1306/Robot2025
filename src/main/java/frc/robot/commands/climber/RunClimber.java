package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.*;

public class RunClimber extends Command {
    private final Climber climber;
    private final double speed;
    private final Direction direction;

    public RunClimber(Climber climber, Direction direction) {
        this.climber = climber;
        this.speed = direction == Direction.FORWARD ? 1 : -1;
        this.direction = direction;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return direction == Direction.FORWARD ? climber.isPastMax() : climber.isPastMin();
    }

    @Override
    public void end(boolean interrupted) {
        climber.setSpeed(0);
    }
}
