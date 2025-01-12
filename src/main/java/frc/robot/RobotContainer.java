// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

    private final AutoFactory autoFactory;
    private final CommandXboxController controller1 = new CommandXboxController(0);
    public final SwerveSubsystem drivebase = new SwerveSubsystem();
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
     */
    private final SwerveInputStream driveAngularVelocity =
            SwerveInputStream.of(drivebase.getSwerveDrive(), () -> controller1.getLeftY() * -1, () -> controller1.getLeftX() * -1)
                    .withControllerRotationAxis(controller1::getRightX).deadband(Constants.LEFT_X_DEADBAND)
                    .scaleTranslation(1).allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
     */
    private final SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(controller1::getRightX,
            controller1::getRightY).headingWhile(true);

    private final Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    private final Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    private final Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    public RobotContainer() {
        Command driveFieldOrientedDirectAngleTest = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-controller1.getLeftY(), 0),
        () -> MathUtil.applyDeadband(-controller1.getLeftX(), 0),
        () -> -controller1.getRightX(),
        () -> -controller1.getRightY());
        
        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleTest);

        autoFactory = new AutoFactory(
                drivebase::getPose, // A function that returns the current robot pose
                drivebase::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
                drivebase::followTrajectory, // The drive subsystem trajectory follower 
                false, // If alliance flipping should be enabled 
                drivebase, // The drive subsystem
                (sample, isStart) -> {System.out.println(sample.getTotalTime());}
        );
        trajectory = Choreo.loadTrajectory("TestPath").get();

        Command odometryAutoCommand = new InstantCommand(() -> drivebase.resetOdometry(trajectory.getInitialSample(false).get().getPose()));
        odometryAutoCommand.ignoringDisable(true).schedule();
    }

    private final Trajectory<?> trajectory;

    public Command getAutonomousCommand() {
        return autoFactory.trajectoryCmd(trajectory).andThen(new InstantCommand(() -> System.out.println("\nFinished\n")));
    }
}
