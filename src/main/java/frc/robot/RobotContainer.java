// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmSetpoints;
import frc.robot.commands.arm.ManualArmControl;
import frc.robot.commands.arm.MoveArmToSetpoint;
import frc.robot.commands.auto.CustomWaitCommand;
import frc.robot.commands.autos.*;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.elevator.ElevatorSetpoints;
import frc.robot.commands.elevator.ManualElevatorControl;
import frc.robot.commands.elevator.MoveElevatorToSetpoint;
import frc.robot.commands.elevator.ZeroElevatorRoutine;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.led.FillLEDColor;
import frc.robot.commands.led.LEDPatterns;
import frc.robot.commands.wrist.ManualWristControl;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.commands.wrist.WristSetpoints;
import frc.robot.subsystems.*;
import frc.robot.util.Utilities;
import lombok.Getter;
import swervelib.SwerveInputStream;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import badgerlog.Dashboard;
import badgerlog.entry.Entry;
import badgerlog.entry.EntryType;

import static edu.wpi.first.units.Units.Inches;

public class RobotContainer {

    private final CommandXboxController controller1 = new CommandXboxController(0);
    private final CommandXboxController controller2 = new CommandXboxController(1);

    private final Wrist wrist = new Wrist();
    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();
    private final LEDSubsystem ledStrip = new LEDSubsystem(Constants.LED_PORT, 0, Constants.LED_COUNT);
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    private final SwerveInputStream driveAngularVelocity =
            SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -controller1.getLeftY(), () -> -controller1.getLeftX())
                    .withControllerRotationAxis(() -> -controller1.getRightX()).deadband(Constants.LEFT_X_DEADBAND)
                    .scaleTranslation(1).scaleRotation(0.75).allianceRelativeControl(true);
    
    private final Command driveRobotOrientedAngularVelocity = drivebase.drive(driveAngularVelocity);
    private final Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    @Entry(key = "Auto/Auto Chooser", type = EntryType.Sendable)
    private static AutoChooser autoChooser = new AutoChooser();
    
    @Entry(key = "Auto/Controller Chooser",type = EntryType.Sendable)
    private static SendableChooser<EventLoop> controllerModeChooser = new SendableChooser<>();

    @Entry(key = "Auto/Auto Wait Time", type = EntryType.Subscriber)
    private static double autoWaitTime = 0;

    public RobotContainer() {
        // UsbCamera camera = CameraServer.startAutomaticCapture();
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
        
        //Autos
        Autos autos = new Autos(drivebase, arm, elevator, intake, wrist);
        autoChooser.addRoutine("Left 2 H", () -> autos.get1CoralL4DriveRoutine("Score Left 2 H"));
        autoChooser.addRoutine("Mid B", () -> autos.get1CoralL4DriveRoutine("Score Mid B"));
        autoChooser.addRoutine("Right 2 C", () -> autos.get1CoralL4DriveRoutine("Score Right 2 C"));

        autoChooser.addRoutine("2 Coral: Left 2", () -> autos.get2CoralDriveRoutine("Score Left 2 H", "Left 2 Intermediate Pickup", "Score Pickup I"));
        autoChooser.addRoutine("2 Coral: Right 2", () -> autos.get2CoralDriveRoutine("Score Right 2 C", "Right 2 Intermediate Pickup", "Score Pickup F"));
        
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

    //    arm.setDefaultCommand(new ArmFromSmartDashboard(arm));
//        wrist.setDefaultCommand(new WristFromSmartDashboard(wrist));
    //    elevator.setDefaultCommand(new ElevatorFromSmartDashboard(elevator));
    }

    public void zeroTargetPositions(){
        Elevator.setTargetHeight(Inches.of(0));
        wrist.setTargetAngle(Rotation2d.kZero);
        arm.setTargetAngle(Rotation2d.kZero);
    }

    public void resetTargetPositions(){
        Elevator.setTargetHeight(Inches.of(0));
        wrist.setTargetAngle(Rotation2d.kZero);
        arm.setTargetAngle(ArmSetpoints.STOW.getAngle());
    }
    
    private final EventLoop fullManualEventLoop = new EventLoop();
    private final EventLoop fullAutomaticEventLoop = new EventLoop();
    private final EventLoop setpointEventLoop = new EventLoop();
    private final EventLoop alternativeEventLoop = new EventLoop();

    public void bindAlternative(){
        bindCommonControls(alternativeEventLoop);
        controller1.a(alternativeEventLoop).whileTrue(drivebase.getReefAutoAlignCommand());
    }
    
    public void bindManual(){
        bindCommonControls(fullManualEventLoop);

        controller1.pov(0, 0, fullManualEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.HORIZONTAL));
        controller1.pov(0, 90, fullManualEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_R));
        controller1.pov(0, 270, fullManualEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_L));

        controller2.back(fullManualEventLoop).toggleOnTrue(new ManualArmControl(arm, controller2::getRightY));
        controller2.y(fullManualEventLoop).toggleOnTrue(new ManualElevatorControl(elevator, controller2::getLeftY));
        controller2.x(fullManualEventLoop).toggleOnTrue(new ManualWristControl(wrist, controller2::getLeftX));
    }

    public void bindSetpoint(){
        bindCommonControls(setpointEventLoop);
        
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
        
        controller2.back(setpointEventLoop).onTrue(new MoveArmToSetpoint(arm, ArmSetpoints.STOW));
        
        controller2.rightTrigger(0.5, setpointEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_R));
        controller2.leftTrigger(0.5, setpointEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.VERTICAL_L));
        controller2.leftBumper(setpointEventLoop).onTrue(new MoveWristToSetpoint(wrist, WristSetpoints.HORIZONTAL));
    }
    
    private int selectedLevel = 1;
    private static boolean wristLeft = true;
    
    //Make sure to implement correctly (use a supplier in an init method)
    @Getter
    private static final boolean overrideSafeMode = false;

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
        placingCommands.put(LevelSelectorKey.ALGAE_REMOVE_L2, new StageRemoveAlgae(elevator, arm, wrist, 2));
        placingCommands.put(LevelSelectorKey.ALGAE_REMOVE_L3, new StageRemoveAlgae(elevator, arm, wrist, 3));
        
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
        scoringCommands.put(LevelSelectorKey.ALGAE_REMOVE_L2, new RemoveAlgae(elevator, arm, wrist, 2));
        scoringCommands.put(LevelSelectorKey.ALGAE_REMOVE_L3, new RemoveAlgae(elevator, arm, wrist, 3));
        
        ConditionalCommandChooser<LevelSelectorKey> scoreWrapper = new ConditionalCommandChooser<>(scoringCommands, this::getLevelSelectorKey);

        controller1.rightTrigger(0.5, fullAutomaticEventLoop).onTrue(scoreWrapper);        
        controller1.leftBumper(fullAutomaticEventLoop).onTrue(new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.STOW, ArmSetpoints.STOW, WristSetpoints.HORIZONTAL));
        
        controller1.a(fullAutomaticEventLoop).onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.GROUND_CORAL, ArmSetpoints.GROUND_CORAL, WristSetpoints.HORIZONTAL)
        );

        controller1.b(fullAutomaticEventLoop).whileTrue(drivebase.getCoralStationAutoAlign());
        
        controller1.x(fullAutomaticEventLoop).onTrue(
                new MoveToolingToSetpoint(elevator, arm, wrist, ElevatorSetpoints.CORAL_STATION, ArmSetpoints.CORAL_STATION, WristSetpoints.HORIZONTAL)
        );

        controller1.rightStick(fullAutomaticEventLoop).whileTrue(drivebase.getReefAutoAlignCommand());

        //slow mode
        controller1.leftTrigger(0.5, fullAutomaticEventLoop).onTrue(drivebase.changeSwerveSpeed(0.2)).onFalse(drivebase.changeSwerveSpeed(1));

        controller2.leftTrigger(0.5, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> wristLeft = false));
        controller2.rightTrigger(0.5, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> wristLeft = true));

        controller2.leftStick(fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 6));
        controller2.rightStick(fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 5));
        controller2.pov(0, 0, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 4));
        controller2.pov(0, 90, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 3));
        controller2.pov(0, 180, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 2));
        controller2.pov(0, 270, fullAutomaticEventLoop).onTrue(new InstantCommand(() -> selectedLevel = 1));
        
        controller2.x(fullAutomaticEventLoop).toggleOnTrue(new ManualArmControl(arm, () -> -controller2.getRightY()));

        controller2.a(fullAutomaticEventLoop).whileTrue(new RunIntake(intake, () -> 1));
        controller2.b(fullAutomaticEventLoop).whileTrue(new RunIntake(intake, () -> -1));

        controller2.back(fullAutomaticEventLoop).whileTrue(new ZeroElevatorRoutine(elevator));
        controller2.rightBumper(fullAutomaticEventLoop).whileTrue(new RunClimber(climber, Constants.Direction.FORWARD)); // climb
        controller2.leftBumper(fullAutomaticEventLoop).whileTrue(new RunClimber(climber, Constants.Direction.REVERSE)); // deploy
    }

    private LevelSelectorKey getLevelSelectorKey(){
        return switch (selectedLevel) {
            case 2 -> wristLeft ? LevelSelectorKey.CORAL_L2_L : LevelSelectorKey.CORAL_L2_R;
            case 3 -> wristLeft ? LevelSelectorKey.CORAL_L3_L : LevelSelectorKey.CORAL_L3_R;
            case 4 -> wristLeft ? LevelSelectorKey.CORAL_L4_L : LevelSelectorKey.CORAL_L4_R;
            case 5 -> LevelSelectorKey.ALGAE_REMOVE_L2;
            case 6 -> LevelSelectorKey.ALGAE_REMOVE_L3;
            default -> LevelSelectorKey.CORAL_L1;
        };
    }

    public static Runnable autoRunnable = null;
    
    public void bindCommonControls(EventLoop loop){
        controller1.start(loop).onTrue(new InstantCommand(drivebase::zeroGyro).ignoringDisable(true));
        controller1.leftStick(loop)
            .onFalse(new InstantCommand(() -> changeDrivebaseDefaultCommand(driveFieldOrientedAngularVelocity)))
            .onTrue(new InstantCommand(() -> changeDrivebaseDefaultCommand(driveRobotOrientedAngularVelocity)));

        controller2.start(loop).onTrue(new InstantCommand(elevator::zeroElevatorMotorPositions).ignoringDisable(true));

        new Trigger(loop, DriverStation::isAutonomousEnabled).whileTrue(new CustomWaitCommand(() -> autoWaitTime).andThen(autoChooser.selectedCommandScheduler()));
        new Trigger(loop, Utilities::isRedAlliance).onChange(new InstantCommand(FieldLocation::recalculateFieldPositions).ignoringDisable(true));

        Dashboard.getAutoResettingButton("Auto/Reset Auto Odometry", loop)
            .and(DriverStation::isDisabled)
            .and(() -> autoRunnable != null)
            .onTrue(new InstantCommand(() -> autoRunnable.run()).ignoringDisable(true));
    }

    private void changeDrivebaseDefaultCommand(Command defaultCommand){
        Utilities.removeAndCancelDefaultCommand(drivebase);
        drivebase.setDefaultCommand(defaultCommand);
    }
    
    public BooleanSupplier isEventLoopScheduled(EventLoop loop){
        return () -> CommandScheduler.getInstance().getActiveButtonLoop().equals(loop);
    }
    
    public void changeEventLoop(EventLoop loop){
        CommandScheduler.getInstance().setActiveButtonLoop(loop);
    }
    
    public void setAllianceLed(){
        FillLEDColor.setAlianceColor(ledStrip).ignoringDisable(true).schedule();
    }
    public void setRainbow(){
        LEDPatterns.setRainbowEffect(ledStrip).ignoringDisable(true).schedule();
    }
    public void setSeizureMode(){
        LEDPatterns.seizureMode(ledStrip).ignoringDisable(true).schedule();
    }
}