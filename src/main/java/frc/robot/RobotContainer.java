// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.arm.ArmFromSmartDashboard;
import frc.robot.commands.autos.FieldLocation;
import frc.robot.commands.elevator.ElevatorFromSmartDashboard;
import frc.robot.commands.led.FillLEDColor;
import frc.robot.commands.wrist.WristFromSmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import swervelib.SwerveInputStream;

public class RobotContainer {

    private final CommandXboxController controller1 = new CommandXboxController(0);
    // private final OperatorContol operatorContol = new OperatorContol(new CommandXboxController(1));

    public final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final LEDSubsystem LEDStrip = new LEDSubsystem(0, Constants.LED_COUNT);
    private final Wrist wrist = new Wrist();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    // private final Intake intake = new Intake();
    
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    private final SwerveInputStream driveAngularVelocity =
            SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -controller1.getLeftY(), () -> -controller1.getLeftX())
                    .withControllerRotationAxis(() -> -controller1.getRightX()).deadband(Constants.LEFT_X_DEADBAND)
                    .scaleTranslation(0.01).allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    private final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> -controller1.getRightX(),
    () -> -controller1.getRightY()).headingWhile(true);


    private final Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    private final Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    private final AutoChooser autoChooser;

    public RobotContainer() {
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
        
        Autos autos = new Autos(drivebase);
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("Test Path", autos::getTestDriveRoutine);
        autoChooser.addRoutine("1 Coral A", autos::get1CoralDriveRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.disabled().onChange(new InstantCommand(FieldLocation::calculateReefPositions));
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        configureBindings();
    }

    public void configureBindings(){
        controller1.start().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
        // controller1.a().onTrue(new DriveToNearestReef(drivebase));
        controller1.a().onTrue(new InstantCommand(() -> elevator.zeroElevatorMotorPositions()).ignoringDisable(true));
        controller1.b().onTrue(
            FillLEDColor.flashTwoColors(LEDStrip, Constants.BLUE, Constants.RED, 1)
        );

    }

    public void toolBindings() {
        
        // controller1.rightTrigger(.5).onTrue(new InstantCommand(() -> {
        //         switch (operatorContol.getSelectedCommand()) {
                    
        //         }
        //     }));

        
    }
}
