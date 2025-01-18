package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Utilities;

public class MoveWristToSetpoint extends Command {

    private final double TOLERANCE = 0.02; //rads

    private Wrist wrist;
    private Rotation2d targetRotation;
    
    public MoveWristToSetpoint(Wrist wrist, WristSetpoint wristSetpoint) {
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
        return Utilities.isEqual(targetRotation.getRadians(), wrist.getCurrentAngle().getRadians(), TOLERANCE);
    }
}