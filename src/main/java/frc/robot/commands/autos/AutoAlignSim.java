
package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Radian;

import badgerlog.entry.Entry;

import badgerlog.entry.EntryType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpersSim;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignSim extends Command {
  private PIDController xController, yController, rotationController;
  
  private boolean direction;
  private Timer noTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;

  public AutoAlignSim(boolean direction, SwerveSubsystem drivebase) {
    xController = new PIDController(2, 0.0, 0);  
    yController = new PIDController(2, 0.0, 0);  
    rotationController = new PIDController(0.8, 0, 1);  
    this.direction = direction; 

    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
   
    this.stopTimer = new Timer();
    this.stopTimer.start();
    
    this.noTagTimer = new Timer();
    this.noTagTimer.start();

    rotationController.setSetpoint(Constants.ROT_SETPOINT);
    rotationController.setTolerance(Constants.ROT_TOLERANCE);

    xController.setSetpoint(Constants.X_SETPOINT);
    xController.setTolerance(Constants.X_TOLERANCE);

    yController.setSetpoint(direction ? Constants.Y_SETPOINT : -Constants.Y_SETPOINT);
    yController.setTolerance(Constants.Y_TOLERANCE);

    tagID = LimelightHelpersSim.getFiducialID();
  }

  @Override
  public void execute() {
    if (LimelightHelpersSim.getTV() && LimelightHelpersSim.getFiducialID() == tagID) {
      this.noTagTimer.reset();
      SmartDashboard.putNumber("TagID", tagID);
      Pose3d robotPose = LimelightHelpersSim.getTargetPose3d_RobotSpace(this.drivebase, new Pose2d(13.86, 5.17, new Rotation2d(0)));

      double xSpeed = xController.calculate(-robotPose.getX());
      double ySpeed = yController.calculate(robotPose.getY()); // Not inverted because WPILIB and Limlight y's are inverted
      double rotValue = rotationController.calculate(-robotPose.getRotation().getZ());
      SmartDashboard.putNumber("targetX", robotPose.getX());
      SmartDashboard.putNumber("targetY", robotPose.getY());
      SmartDashboard.putNumber("targetRot", robotPose.getRotation().getZ());
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      drivebase.drive(new ChassisSpeeds(xSpeed, ySpeed, rotValue));

      if(
        !rotationController.atSetpoint() ||
        !yController.atSetpoint() ||
        !xController.atSetpoint()
        ) 
      {
        stopTimer.reset();
      }
    } else { 
      drivebase.drive(new ChassisSpeeds(0, 0, 0));
    }
      
    
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return this.noTagTimer.hasElapsed(Constants.NO_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.CORRECT_POSE_WAIT_TIME);
  }
}