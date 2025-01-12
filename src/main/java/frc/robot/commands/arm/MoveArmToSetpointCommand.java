package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToSetpointCommand extends Command {
    
    private Arm arm;
    private final Rotation2d targetRotation;


    public MoveArmToSetpointCommand(Arm arm, ArmSetpoints armSetpoint) {
        this.arm = arm;
        this.targetRotation = armSetpoint.rotation;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setTargetAngle(targetRotation);
    }
}
