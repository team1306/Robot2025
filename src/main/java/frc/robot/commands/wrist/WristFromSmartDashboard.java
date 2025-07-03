package frc.robot.commands.wrist;

import badgerlog.entry.Entry;
import badgerlog.entry.EntryType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class WristFromSmartDashboard extends Command {
    private final Wrist wrist;

    @Entry(EntryType.Subscriber)
    private static Rotation2d targetRotation = Rotation2d.kZero;
    
    /**
     * Sets the wrist angle
     */
    public WristFromSmartDashboard(Wrist wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setTargetAngle(targetRotation);
    }   
}