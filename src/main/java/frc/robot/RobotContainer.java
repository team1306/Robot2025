// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.autos.*;
import frc.robot.commands.drive.RotateToRotation;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.ManualElevatorControl;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.elevator.ZeroElevatorRoutine;
import frc.robot.commands.intake.IntakeCoral;
import frc.robot.commands.intake.SpitCoral;
import frc.robot.commands.led.FillLEDColor;
import frc.robot.commands.led.LEDPatterns;
import frc.robot.commands.wrist.ManualWristControl;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.*;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;
import frc.robot.util.Dashboard.PutValue;
import lombok.Getter;
import swervelib.SwerveInputStream;

import java.util.HashMap;

public class RobotContainer {

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);
    
    public final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final LEDSubsystem LEDStrip = new LEDSubsystem(0, Constants.LED_COUNT);
    private final Wrist wrist = new Wrist();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    
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

    private final AutoChooser autoChooser;
    private final SendableChooser<EventLoop> controllerModeChooser = new SendableChooser<>();

    public RobotContainer() {
        DashboardHelpers.addUpdateClass(this);
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
        
        //Autos
        Autos autos = new Autos(drivebase);
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("Test Path", autos::getTestDriveRoutine);
        autoChooser.addRoutine("1 Coral A", autos::get1CoralDriveRoutine);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //Controller Chooser
        bindAlternative();
        bindAutomatic();
        bindManual();
        bindSetpoint();
        changeEventLoop(alternativeEventLoop);

        controllerModeChooser.addOption("Full Auto", fullAutomaticEventLoop);
        controllerModeChooser.addOption("Setpoints", setpointEventLoop);
        controllerModeChooser.addOption("Manual", fullManualEventLoop);

        controllerModeChooser.setDefaultOption("Alternative", alternativeEventLoop);
        controllerModeChooser.onChange(this::changeEventLoop);

        SmartDashboard.putData("Controller Binding Chooser", controllerModeChooser);
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
        controller1.rightTrigger(.5).onTrue(new PlaceCoral(elevator, arm, wrist, intake, () -> MoveElevatorToSetpoint.getLastLevel()));

        controller2.a().onTrue(
            new MoveToolingToSetpoint(arm, elevator, wrist,
                ElevatorSetpoints.STOW,
                ArmSetpoints.GROUND_CORAL,
                WristSetpoints.HORIZONTAL
            )
            .raceWith(
                new IntakeCoral(intake)
            ).andThen(
                new MoveToolingToSetpoint(arm, elevator, wrist,
                    ElevatorSetpoints.STOW,
                    ArmSetpoints.STOW,
                    WristSetpoints.HORIZONTAL
                )
            )
        );

        controller2.x().onTrue(
            new MoveToolingToSetpoint(arm, elevator, wrist,
                ElevatorSetpoints.CORAL_STATION,
                ArmSetpoints.CORAL_STATION,
                WristSetpoints.HORIZONTAL
            )
        );

        controller2.povUp().onTrue(
            new MoveToolingToSetpoint(arm, elevator, wrist,
                ElevatorSetpoints.CORAL_L4,
                ArmSetpoints.CORAL_STATION,
                WristSetpoints.VERTICAL
            )
        );

        controller2.povRight().onTrue(
            new MoveToolingToSetpoint(arm, elevator, wrist,
                ElevatorSetpoints.CORAL_L3,
                ArmSetpoints.CORAL_STATION,
                WristSetpoints.VERTICAL
            )
        );

        controller2.povLeft().onTrue(
            new MoveToolingToSetpoint(arm, elevator, wrist,
                ElevatorSetpoints.CORAL_L2,
                ArmSetpoints.CORAL_STATION,
                WristSetpoints.VERTICAL
            )
        );
        
        controller1.leftStick(fullAutomaticEventLoop).onTrue(new RotateToRotation(drivebase, () -> drivebase.getPose().nearest(FieldLocation.reefLocations).getRotation()));
        controller1.rightStick(fullAutomaticEventLoop).onTrue(new RotateToRotation(drivebase, () -> drivebase.getPose().nearest(FieldLocation.coralStationLocations).getRotation()));
        
        controller1.leftTrigger(0.5, fullAutomaticEventLoop).onTrue(new SpitCoral(intake));

        controller2.pov(0, 0, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 4));
        controller2.pov(0, 90, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 3));
        controller2.pov(0, 180, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 2));
        controller2.pov(0, 270, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 1));
        
        controller2.x(fullAutomaticEventLoop).toggleOnTrue(new ManualElevatorControl(elevator, controller2::getLeftY));
        controller2.back().whileTrue(new ZeroElevatorRoutine(elevator));
    }

    public void bindCommonControls(EventLoop loop){
        controller1.start(loop).onTrue(new InstantCommand(drivebase::zeroGyro).ignoringDisable(true));
        controller2.start(loop).onTrue(new InstantCommand(elevator::zeroElevatorMotorPositions).ignoringDisable(true));

        new Trigger(loop, DriverStation::isAutonomousEnabled).whileTrue(autoChooser.selectedCommandScheduler());
        new Trigger(loop, DriverStation::isDisabled).onChange(new InstantCommand(FieldLocation::calculateReefPositions));
    }
    
    public void changeEventLoop(EventLoop loop){
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }
}
