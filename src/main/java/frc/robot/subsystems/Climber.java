package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Dashboard.DashboardHelpers;

public class Climber extends SubsystemBase {
    TalonFX driveKraken;
    public Climber() {
        DashboardHelpers.addUpdateClass(this);
    }
}
