package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class MoveWristToSetpointCommand extends Command {

    private Wrist wrist;
    
    private Rotation2d targetRotation;
    
    public MoveWristToSetpointCommand(Wrist wrist, WristSetpoints wristSetpoint) {
        this.targetRotation = wristSetpoint.rotation;
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setTargetRotation(targetRotation);
    }

    @Override
    public boolean isFinished() {
        return false; //only ends when interrupted.
    }

}