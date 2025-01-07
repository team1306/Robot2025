// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private CommandXboxController p1Controller = new CommandXboxController(0);
  
  private SwerveSubsystem drivebase = new SwerveSubsystem();

  private final AutoFactory autoFactory;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  private SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(), () -> p1Controller.getLeftY() * -1, () -> p1Controller.getLeftX() * -1)
      .withControllerRotationAxis(p1Controller::getRightX).deadband(Constants.LEFT_X_DEADBAND)
      .scaleTranslation(0.8).allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  private SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(p1Controller::getRightX,
    p1Controller::getRightY).headingWhile(true);

  private Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  private Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  public RobotContainer(){
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    autoFactory = new AutoFactory(
      drivebase::getPose, // A function that returns the current robot pose
      drivebase::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
      drivebase::followTrajectory, // The drive subsystem trajectory follower 
      true, // If alliance flipping should be enabled 
      drivebase // The drive subsystem
        );
  }

  public Command getAutonomousCommand(){
    return autoFactory.trajectoryCmd("TestPath");
  }
}
