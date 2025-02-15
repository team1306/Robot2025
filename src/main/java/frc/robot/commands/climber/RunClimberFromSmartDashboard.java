package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;

public class RunClimberFromSmartDashboard extends Command {
    private final Climber climber;

    @GetValue
    private double speed = 0;

    public RunClimberFromSmartDashboard(Climber climber) {
        this.climber = climber;
        DashboardHelpers.addUpdateClass(this);
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
