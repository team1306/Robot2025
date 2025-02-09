package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Getter;

public class OperatorContol {
    
    final CommandXboxController controller;

    @Getter
    private OperatorCommand selectedCommand;

    public OperatorContol(CommandXboxController controller) {
        this.controller = controller;
        
        controller.povUp().onTrue(new InstantCommand(() -> selectedCommand = OperatorCommand.CORAL_L1));
        controller.povRight().onTrue(new InstantCommand(() -> selectedCommand = OperatorCommand.CORAL_L2));
        controller.povDown().onTrue(new InstantCommand(() -> selectedCommand = OperatorCommand.CORAL_L3));
        controller.povLeft().onTrue(new InstantCommand(() -> selectedCommand = OperatorCommand.CORAL_L4));
    }
}

enum OperatorCommand {
    CORAL_L1,
    CORAL_L2,
    CORAL_L3,
    CORAL_L4,
    CORAL_STATION;
}