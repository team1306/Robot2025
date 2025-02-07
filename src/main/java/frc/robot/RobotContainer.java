// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.autos.FieldLocation;
import frc.robot.commands.led.FillLEDColor;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

    private final AutoFactory autoFactory;
    private final CommandXboxController controller1 = new CommandXboxController(0);
    public final SwerveSubsystem drivebase = new SwerveSubsystem();
    public final LEDSubsystem LEDStrip = new LEDSubsystem(0, Constants.LED_COUNT);
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

    public RobotContainer() {
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);

        autoFactory = new AutoFactory(
                drivebase::getPose, // A function that returns the current robot pose
                drivebase::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
                drivebase::followTrajectory, // The drive subsystem trajectory follower 
                false, // If alliance flipping should be enabled 
                drivebase // The drive subsystem
        );

        autoChooser = new AutoChooser();

        // Add options to the chooser
        autoChooser.addRoutine("Choreo", this::getDriveRoutine);
        autoChooser.addCmd("Pathplanner", this::getAutonomousCommand);
        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
        configureBindings();
    }

    public void configureBindings(){
        controller1.start().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
        controller1.a().onTrue(
            drivebase.driveToReef(
                FieldLocation.H, 
                FieldLocation.getIntermediatePoseFromFinal(FieldLocation.H))
            .andThen( 
                new InstantCommand(() -> drivebase.getSwerveController().lastAngleScalar = FieldLocation.H.getRotation().getRadians())));
        controller1.b().onTrue(
            FillLEDColor.fillColor(LEDStrip, Constants.RED)
        );

    }

    public Command getAutonomousCommand() {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Example Path");
            drivebase.resetOdometry(path.getStartingHolonomicPose().get());
            return new InstantCommand(() -> drivebase.resetOdometry(path.getStartingHolonomicPose().get())).andThen(AutoBuilder.followPath(path));
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new InstantCommand();
    }

    public AutoRoutine getDriveRoutine(){
        AutoRoutine routine = autoFactory.newRoutine("DriveRoutine");
        AutoTrajectory pickupTraj = routine.trajectory("TestPath");

        routine.active().onTrue(
        Commands.sequence(
            pickupTraj.resetOdometry(),
            pickupTraj.cmd()
        ));

        return routine;
    }
}
