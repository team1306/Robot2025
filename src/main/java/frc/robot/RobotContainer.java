// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Direction;
import frc.robot.commands.arm.ArmFromSmartDashboard;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.autos.*;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.drive.RotateToRotation;
import frc.robot.commands.elevator.ElevatorFromSmartDashboard;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.ManualElevatorControl;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.elevator.ZeroElevatorRoutine;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.led.FillLEDColor;
import frc.robot.commands.led.LEDPatterns;
import frc.robot.commands.wrist.ManualWristControl;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristFromSmartDashboard;
import frc.robot.commands.wrist.WristSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.*;
import frc.robot.util.Utilities;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.PutValue;
import lombok.Getter;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Inches;

import java.util.HashMap;

public class RobotContainer {

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);
    
    public final SwerveSubsystem drivebase = new SwerveSubsystem();
    // private final LEDSubsystem LEDStrip = new LEDSubsystem(Constants.LED_PORT, 0, Constants.LED_COUNT);
    // private final LEDSubsystem chainLEDStrip = new LEDSubsystem(Constants.CHAIN_LED_PORT, 0, Constants.CHAIN_LED_COUNT);
    private final Wrist wrist = new Wrist();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();
    
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    private final SwerveInputStream driveAngularVelocity =
            SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -controller1.getLeftY(), () -> -controller1.getLeftX())
                    .withControllerRotationAxis(() -> -controller1.getRightX()).deadband(Constants.LEFT_X_DEADBAND)
                    .scaleTranslation(0.5).allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    private final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> -controller1.getRightX(),
    () -> -controller1.getRightY()).headingWhile(true);


    private final Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    private final Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


    private final AutoChooser autoChooser;
    private final SendableChooser<EventLoop> controllerModeChooser = new SendableChooser<>();

    public RobotContainer() {
        DashboardHelpers.addUpdateClass(this);
        UsbCamera camera = CameraServer.startAutomaticCapture();
        // camera.setFPS(10);
        // camera.setResolution(100, 100);
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        
        //Autos
        Autos autos = new Autos(drivebase, arm, elevator, intake, wrist);
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("1 Coral A", () -> {
            AutoRoutine routine = autos.get1CoralDriveRoutine();
            Robot.runnable = () -> drivebase.resetOdometry(autos.coralPath.getInitialPose().get());
            return routine;
        });
        autoChooser.addRoutine("Leave Red 2", autos::getLeaveRoutine);

        SmartDashboard.putData("Auto Chooser", autoChooser);

        //Controller Chooser
        bindAlternative();
        bindAutomatic();
        bindManual();
        bindSetpoint();
        changeEventLoop(alternativeEventLoop);

        controllerModeChooser.setDefaultOption("Full Auto", fullAutomaticEventLoop);
        controllerModeChooser.addOption("Setpoints", setpointEventLoop);
        controllerModeChooser.addOption("Manual", fullManualEventLoop);

        controllerModeChooser.addOption("Alternative", alternativeEventLoop);
        controllerModeChooser.onChange(this::changeEventLoop);

        SmartDashboard.putData("Controller Binding Chooser", controllerModeChooser);
        
        // arm.setDefaultCommand(new ArmFromSmartDashboard(arm));
        // wrist.setDefaultCommand(new WristFromSmartDashboard(wrist));
        // elevator.setDefaultCommand(new ElevatorFromSmartDashboard(elevator));
    }

    public void zeroTargetPositions(){
        elevator.setTargetHeight(Inches.of(0));
        wrist.setTargetAngle(Rotation2d.kZero);
        arm.setTargetAngle(Rotation2d.kZero);
    }

    public void resetTargetPositions(){
        // elevator.setTargetHeight(elevator.getCurrentHeight());
        // wrist.setTargetAngle(wrist.getCurrentAngle());
        // arm.setTargetAngle(arm.getCurrentAngle());
    }
    
    private final EventLoop fullManualEventLoop = new EventLoop();
    private final EventLoop fullAutomaticEventLoop = new EventLoop();
    private final EventLoop setpointEventLoop = new EventLoop();
    private final EventLoop alternativeEventLoop = new EventLoop();

    /**
     * Change these bindings for any testing needed
     */
    public void bindAlternative(){
        bindCommonControls(alternativeEventLoop);

        
        controller1.a(setpointEventLoop).toggleOnTrue(new RunIntake(intake, () -> 1));
        controller1.b(setpointEventLoop).toggleOnTrue(new RunIntake(intake, () -> -1));
    }
    
    public void bindManual(){
        bindCommonControls(fullManualEventLoop);

        controller1.pov(0, 0, fullManualEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.HORIZONTAL));
        controller1.pov(0, 90, fullManualEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_R));
        controller1.pov(0, 270, fullManualEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_L));

        // controller2.back(fullManualEventLoop).toggleOnTrue(new ManualArmControl(arm, controller2::getRightY));
        controller2.y(fullManualEventLoop).toggleOnTrue(new ManualElevatorControl(elevator, controller2::getLeftY));
        // controller2.x(fullManualEventLoop).toggleOnTrue(new ManualWristControl(wrist, controller2::getLeftX));

        // controller1.a(fullManualEventLoop).toggleOnTrue(new RunIntake(intake, () -> 1));
        // controller1.b(fullManualEventLoop).toggleOnTrue(new RunIntake(intake, () -> -1));

        // controller1.rightBumper(fullManualEventLoop).whileTrue(new RunClimber(climber, Direction.REVERSE)); // deploy
        // controller1.leftBumper(fullManualEventLoop).whileTrue(new RunClimber(climber, Direction.FORWARD)); // climb

    }

    public void bindSetpoint(){
        bindCommonControls(setpointEventLoop);

        // controller1.leftStick(setpointEventLoop).onTrue(new RotateToRotation(drivebase, () -> drivebase.getPose().nearest(FieldLocation.reefLocations).getRotation()));
        // controller1.rightStick(setpointEventLoop).onTrue(new RotateToRotation(drivebase, () -> drivebase.getPose().nearest(FieldLocation.coralStationLocations).getRotation()));
        
        controller1.pov(0, 0, setpointEventLoop).onTrue(new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.CORAL_L4));
        controller1.pov(0, 90, setpointEventLoop).onTrue(new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.CORAL_L3));
        controller1.pov(0, 180, setpointEventLoop).onTrue(new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.CORAL_L2));
        controller1.pov(0, 270, setpointEventLoop).onTrue(new MoveElevatorToSetpoint(elevator, ElevatorSetpoints.CORAL_L1));

        controller1.a(setpointEventLoop).toggleOnTrue(new RunIntake(intake, () -> 1));
        controller1.b(setpointEventLoop).toggleOnTrue(new RunIntake(intake, () -> -1));

        controller2.pov(0, 0, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_L4));
        controller2.pov(0, 90, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_L3));
        controller2.pov(0, 180, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_L2));
        controller2.pov(0, 270, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_L1));

        // controller2.pov(0, 45, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.HOVER_L4));
        // controller2.pov(0, 135, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.HOVER_L2));
        // controller2.pov(0, 225, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.GROUND_CORAL));
        // controller2.pov(0, 315, setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.CORAL_STATION));
        
        controller2.back(setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.STOW));
        
        controller2.rightTrigger(0.5, setpointEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_R));
        controller2.leftTrigger(0.5, setpointEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_L));
        controller2.leftBumper(setpointEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.HORIZONTAL));

        // controller1.rightBumper(setpointEventLoop).whileTrue(new RunClimber(climber, Direction.REVERSE)); // deploy
        // controller1.leftBumper(setpointEventLoop).whileTrue(new RunClimber(climber, Direction.FORWARD)); // climb
    }
    
    private int selectedLevel = 1;
    private boolean wristLeft = true;
    
    //Make sure to implement correctly (use a supplier in an init method)
    @Getter
    private static boolean overrideSafeMode = false;

    public void bindAutomatic(){
        bindCommonControls(fullAutomaticEventLoop);
        
        //You have to wrap the command options in another command because Java compiles conditionals returning values at compile time, not runtime
        //Alternative option would be to decorate 4+ commands with the .onlyIf() and .alongWith() decorators (not great)
        HashMap<LevelSelectorKey, Command> placingCommands = new HashMap<>();
        placingCommands.put(LevelSelectorKey.CORAL_L1, new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_L1, ArmSetpoints.CORAL_L1, WristSetpoints.HORIZONTAL, true));
        placingCommands.put(LevelSelectorKey.CORAL_L2_L, new PlaceCoral(elevator, arm, wrist, 2, WristSetpoints.VERTICAL_L));
        placingCommands.put(LevelSelectorKey.CORAL_L2_R, new PlaceCoral(elevator, arm, wrist, 2, WristSetpoints.VERTICAL_R));
        placingCommands.put(LevelSelectorKey.CORAL_L3_L, new PlaceCoral(elevator, arm, wrist, 3, WristSetpoints.VERTICAL_L));
        placingCommands.put(LevelSelectorKey.CORAL_L3_R, new PlaceCoral(elevator, arm, wrist, 3, WristSetpoints.VERTICAL_R));
        placingCommands.put(LevelSelectorKey.CORAL_L4_L, new PlaceCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_L));
        placingCommands.put(LevelSelectorKey.CORAL_L4_R, new PlaceCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_R));
        
        ConditionalCommandChooser<LevelSelectorKey> placeWrapper = new ConditionalCommandChooser<>(placingCommands, this::getLevelSelectorKey);
        controller1.rightBumper(fullAutomaticEventLoop).onTrue(placeWrapper);
        
        HashMap<LevelSelectorKey, Command> scoringCommands = new HashMap<>();
        scoringCommands.put(LevelSelectorKey.CORAL_L1, new RunIntake(intake, () -> 0.25));
        scoringCommands.put(LevelSelectorKey.CORAL_L2_L, new DropCoral(elevator, arm, wrist, 2, WristSetpoints.VERTICAL_L));
        scoringCommands.put(LevelSelectorKey.CORAL_L2_R, new DropCoral(elevator, arm, wrist, 2, WristSetpoints.VERTICAL_R));
        scoringCommands.put(LevelSelectorKey.CORAL_L3_L, new DropCoral(elevator, arm, wrist, 3, WristSetpoints.VERTICAL_L));
        scoringCommands.put(LevelSelectorKey.CORAL_L3_R, new DropCoral(elevator, arm, wrist, 3, WristSetpoints.VERTICAL_R));
        scoringCommands.put(LevelSelectorKey.CORAL_L4_L, new DropCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_L));
        scoringCommands.put(LevelSelectorKey.CORAL_L4_R, new DropCoral(elevator, arm, wrist, 4, WristSetpoints.VERTICAL_R));

        ConditionalCommandChooser<LevelSelectorKey> scoreWrapper = new ConditionalCommandChooser<>(scoringCommands, this::getLevelSelectorKey);

        controller1.rightTrigger(0.5, fullAutomaticEventLoop).onTrue(scoreWrapper);        
        controller1.leftBumper(fullAutomaticEventLoop).onTrue(new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.STOW, ArmSetpoints.STOW, WristSetpoints.HORIZONTAL));
        
        controller1.a(fullAutomaticEventLoop).onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.GROUND_CORAL, ArmSetpoints.GROUND_CORAL, WristSetpoints.HORIZONTAL)
        );
        
        controller1.x(fullAutomaticEventLoop).onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_STATION, ArmSetpoints.CORAL_STATION, WristSetpoints.HORIZONTAL)
        );
        
        // controller1.leftStick(fullAutomaticEventLoop).onTrue(new RotateToRotation(drivebase, () -> drivebase.getPose().nearest(FieldLocation.reefLocations).getRotation()));
        // controller1.rightStick(fullAutomaticEventLoop).onTrue(new RotateToRotation(drivebase, () -> drivebase.getPose().nearest(FieldLocation.coralStationLocations).getRotation()));
        
        //slow mode
        controller1.leftTrigger(0.5, fullAutomaticEventLoop).onTrue(drivebase.changeSwerveSpeed(0.25)).onFalse(drivebase.changeSwerveSpeed(1));

        controller2.leftTrigger(0.5, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> wristLeft = true));
        controller2.rightTrigger(0.5, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> wristLeft = false));

        controller2.pov(0, 0, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 4));
        controller2.pov(0, 90, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 3));
        controller2.pov(0, 180, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 2));
        controller2.pov(0, 270, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 1));
        
        controller2.x(fullAutomaticEventLoop).toggleOnTrue(new ManualArmControl(arm, () -> -controller2.getRightY()));

        controller2.a(fullAutomaticEventLoop).whileTrue(new RunIntake(intake, () -> 1));
        controller2.b(fullAutomaticEventLoop).whileTrue(new RunIntake(intake, () -> -1));

        controller2.back().whileTrue(new ZeroElevatorRoutine(elevator));
        controller2.rightBumper(fullAutomaticEventLoop).whileTrue(new RunClimber(climber, Direction.FORWARD)); // climb
        controller2.leftBumper(fullAutomaticEventLoop).whileTrue(new RunClimber(climber, Direction.REVERSE)); // deploy
    }

    private LevelSelectorKey getLevelSelectorKey(){
        switch(selectedLevel){
            default:
                return LevelSelectorKey.CORAL_L1;
            case 2:
                return wristLeft ? LevelSelectorKey.CORAL_L2_L : LevelSelectorKey.CORAL_L2_R;
            case 3:
                return wristLeft ? LevelSelectorKey.CORAL_L3_L : LevelSelectorKey.CORAL_L3_R;
            case 4:
                return wristLeft ? LevelSelectorKey.CORAL_L4_L : LevelSelectorKey.CORAL_L4_R;
        }
    }

    private boolean useAngularVelocity = true;
    
    public void bindCommonControls(EventLoop loop){
        controller1.start(loop).onTrue(new InstantCommand(drivebase::zeroGyro).ignoringDisable(true));
        controller1.back(loop).onTrue(new InstantCommand(() -> {
            Utilities.removeAndCancelDefaultCommand(drivebase);
            useAngularVelocity = !useAngularVelocity;
            drivebase.setDefaultCommand(useAngularVelocity ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngle);
            drivebase.getSwerveDrive().setHeadingCorrection(!useAngularVelocity);
        }));
        
        controller2.start(loop).onTrue(new InstantCommand(elevator::zeroElevatorMotorPositions).ignoringDisable(true));

        new Trigger(loop, DriverStation::isAutonomousEnabled).whileTrue(autoChooser.selectedCommandScheduler());
        new Trigger(loop, DriverStation::isDisabled).onChange(new InstantCommand(FieldLocation::calculateReefPositions));
    }
    
    public void changeEventLoop(EventLoop loop){
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }

    public void alianceLEDs() {
        // FillLEDColor.fillColor(LEDStrip, Constants.LED_OFF).schedule();
        // FillLEDColor.fillColor(LEDStrip, Utilities.isRedAlliance() ? Constants.RED : Constants.BLUE).ignoringDisable(true).schedule();
    }
    public void enableLEDs() {
        // LEDPatterns.setRainbowEffect(LEDStrip);
    }
    public void chainLeds() {
        // LEDPatterns.setRainbowEffect(chainLEDStrip);
    }
}