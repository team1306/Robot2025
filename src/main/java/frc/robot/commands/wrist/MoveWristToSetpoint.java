package frc.robot.commands.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Utilities;

public class MoveWristToSetpoint extends Command {

    private final double TOLERANCE = 0.02; //rads

    private final Wrist wrist;
    private final Rotation2d targetRotation;
    private final boolean finishWhenDone;

    /**
     * Sets the wrist angle
     * @param wristSetpoint setpoint for the wrist to go to
     */
    public MoveWristToSetpoint(Wrist wrist, WristSetpoint wristSetpoint, boolean finishWhenDone) {
        this.targetRotation = wristSetpoint.getAngle();
        this.wrist = wrist;
        this.finishWhenDone = finishWhenDone;
        addRequirements(wrist);
    }

    public MoveWristToSetpoint(Wrist wrist, WristSetpoint wristSetpoint) {
        this(wrist, wristSetpoint, true);
    }

    @Override
    public void execute() {
        wrist.setTargetAngle(targetRotation);
    }

    @Override
    public boolean isFinished() {
        return finishWhenDone && Utilities.isEqual(targetRotation.getRadians(), wrist.getCurrentAngle().getRadians(), TOLERANCE);
    }
}