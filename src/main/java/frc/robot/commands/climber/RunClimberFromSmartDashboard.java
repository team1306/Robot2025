package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.util.dashboardv3.entry.Entry;
import frc.robot.util.dashboardv3.entry.EntryType;

public class RunClimberFromSmartDashboard extends Command {
    private final Climber climber;

    @Entry(type = EntryType.Subscriber)
    private static double speed = 0;

    public RunClimberFromSmartDashboard(Climber climber) {
        this.climber = climber;
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
