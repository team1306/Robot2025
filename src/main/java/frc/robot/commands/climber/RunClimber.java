package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.*;

public class RunClimber extends Command {
    private final Climber climber;
    private final double speed;

    public RunClimber(Climber climber, Direction direction) {
        this.climber = climber;
        this.speed = direction == Direction.FORWARD ? 1 : -1;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.setTargetSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setTargetSpeed(0);
    }
}
