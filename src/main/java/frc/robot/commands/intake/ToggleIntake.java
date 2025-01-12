package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.DoubleSupplier;

public class ToggleIntake extends Command{
    
    private final Intake intake;
    private final DoubleSupplier speed;

    public ToggleIntake(Intake intake, DoubleSupplier speed){
        this.intake = intake;
        this.speed = speed;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setTargetSpeed(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTargetSpeed(0);
    }
}