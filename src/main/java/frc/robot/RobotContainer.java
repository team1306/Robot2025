// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swervedrive.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private CommandXboxController p1Controller = new CommandXboxController(0);
  private SwerveSubsystem drivebase = new SwerveSubsystem();
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase, () -> -MathUtil.applyDeadband(p1Controller.getLeftY(),
    Constants.LEFT_Y_DEADBAND), () -> -MathUtil.applyDeadband(p1Controller.getLeftX(), Constants.LEFT_X_DEADBAND),
    () -> -MathUtil.applyDeadband(p1Controller.getRightX(), Constants.RIGHT_X_DEADBAND),
    p1Controller.getHID()::getYButtonPressed, p1Controller.getHID()::getAButtonPressed,
    p1Controller.getHID()::getXButtonPressed, p1Controller.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(drivebase.getSwerveDrive(), () -> p1Controller.getLeftY() * -1, () -> p1Controller.getLeftX() * -1)
      .withControllerRotationAxis(p1Controller::getRightX).deadband(Constants.LEFT_X_DEADBAND)
      .scaleTranslation(0.8).allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(p1Controller::getRightX,
    p1Controller::getRightY).headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  public RobotContainer(){
    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
  }

  public Command getAutonomousCommand(){
    return drivebase.getAutoCommand();
  }
}
