package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Dashboard.DashboardHelpers;
import frc.robot.util.Dashboard.GetValue;

import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;


public class WheelRadiusCharacterization extends Command {
  private static double characterizationSpeed = 1;
  private final double driveRadius;
  private final DoubleSupplier gyroYawRadsSupplier;
  @RequiredArgsConstructor
  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;
  }

  private final SwerveSubsystem drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(SwerveSubsystem drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    DashboardHelpers.addUpdateClass(this);
    driveRadius = drive.getSwerveDriveConfiguration().getDriveBaseRadiusMeters();
    gyroYawRadsSupplier = () -> drive.getGyroAngle().getRadians();
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPositions();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    final double heading = omegaLimiter.calculate(omegaDirection.value * characterizationSpeed);
    drive.drive(new ChassisSpeeds(0, 0, heading));

    // Get yaw and wheel positions
    final double currRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads += MathUtil.
    angleModulus(currRads - lastGyroYawRads);
    lastGyroYawRads = currRads;
    double averageWheelPosition = 0.0;
    double[] wheelPositiions = drive.getWheelRadiusCharacterizationPositions();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositiions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;
    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    SmartDashboard.putNumber("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    SmartDashboard.putNumber("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    SmartDashboard.putNumber("Drive/RadiusCharacterization/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
  
  }

  @Override
  public void end(boolean interrupted) {
    drive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}