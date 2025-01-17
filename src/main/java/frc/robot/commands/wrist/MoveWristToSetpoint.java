package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class MoveWristToSetpoint extends Command {

    private Wrist wrist;
    private Rotation2d targetRotation;
    
    public MoveWristToSetpoint(Wrist wrist, WristSetpoints wristSetpoint) {
        this.targetRotation = wristSetpoint.rotation;
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wrist.setTargetAngle(targetRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}