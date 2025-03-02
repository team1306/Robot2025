package frc.robot.commands.intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.DoubleSupplier;

public class RunIntake extends Command {
    
    private final Intake intake;
    private final DoubleSupplier speed;

    /**
     * Sets the speed/direction of the intake
     * @param speed speed value for the intake
     */
    public RunIntake(Intake intake, DoubleSupplier speed){
        this.intake = intake;
        this.speed = speed;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setTargetSpeed(speed.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setTargetSpeed(0);
    }
}