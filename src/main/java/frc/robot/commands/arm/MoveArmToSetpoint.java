package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToSetpoint extends Command {
    private final Arm arm;
    private final Rotation2d targetRotation;

    private final boolean finishWhenDone;

    /**
     * Sets the target postiton for the arm
     *
     * @param armSetpoint moves the arm to the provided setpoint
     */
    public MoveArmToSetpoint(Arm arm, ArmSetpoint armSetpoint, boolean finishWhenDone) {
        this.arm = arm;
        this.targetRotation = armSetpoint.getAngle();

        this.finishWhenDone = finishWhenDone;

        addRequirements(arm);
    }

    public MoveArmToSetpoint(Arm arm, ArmSetpoint armSetpoint) {
        this(arm, armSetpoint, true);
    }

    @Override
    public void execute() {
        arm.setTargetAngle(targetRotation);
    }

    @Override
    public boolean isFinished() {
        return finishWhenDone && arm.atSetpoint();
    }
}
