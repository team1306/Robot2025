// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;

import static frc.robot.Constants.*;

public class RobotContainer {
  final CommandXboxController controller1 = new CommandXboxController(0); // Creates an XboxController on port 1.
  private final CommandXboxController controller2 = new CommandXboxController(1); // Creates an XboxController on port 1.
  @GetValue
  public double mult = 0.5;
  final SwerveSubsystem drivebase;
  public RobotContainer() {
    DashboardHelpers.addUpdateClass(this);
    drivebase = new SwerveSubsystem();

    configureBindings();
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-controller1.getLeftY() * mult, 0),
        () -> MathUtil.applyDeadband(-controller1.getLeftX() * mult, 0),
        () -> MathUtil.applyDeadband(-controller1.getRightX(), 0.25),
        () -> MathUtil.applyDeadband(-controller1.getRightY(), .025));

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-controller1.getLeftY() * mult, LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-controller1.getLeftX() * mult, LEFT_X_DEADBAND),
        () -> -controller1.getRightX());
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(controller1.getLeftY() * 0.01, LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(controller1.getLeftX() * 0.01, LEFT_X_DEADBAND),
        () -> controller1.getRawAxis(2));
    Command pushRobot = drivebase.driveCommand(()-> 0, ()->0, ()-> 0);
    // drivebase.setDefaultCommand(driveCommand);


    // drivebase.setDefaultCommand(
    //     !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    // drivebase.setMotorBrake(false);
  }

  /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
  private void configureBindings() {
    controller1.a().toggleOnTrue(new WheelRadiusCharacterization(drivebase, Direction.COUNTER_CLOCKWISE));
    controller1.povUp().onTrue(drivebase.aimAtSetpoint(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(1)));
    controller1.povRight().onTrue(drivebase.aimAtSetpoint(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(1)));
    controller1.povDown().onTrue(drivebase.aimAtSetpoint(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1)));
    controller1.povLeft().onTrue(drivebase.aimAtSetpoint(Rotation2d.fromDegrees(270), Rotation2d.fromDegrees(1)));

    controller1.start().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
  }

  public Command getAutonomousCommand() {
    return drivebase.getAutoCommand();
  }
}
