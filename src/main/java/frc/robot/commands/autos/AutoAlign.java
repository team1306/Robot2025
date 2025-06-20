
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

public class AutoAlign extends Command {
  private PIDController xController, yController, rotationController;
  
  private boolean direction;
  private Timer noTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;

  public AutoAlign(boolean direction, SwerveSubsystem drivebase) {
    xController = new PIDController(2, 0.0, 0);  
    yController = new PIDController(2, 0.0, 0);  
    rotationController = new PIDController(2, 0, 0);  
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

    tagID = LimelightHelpers.getFiducialID(Constants.LIMELIGHT_4_NAME);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV(Constants.LIMELIGHT_4_NAME) && LimelightHelpers.getFiducialID(Constants.LIMELIGHT_4_NAME) == tagID) {
      this.noTagTimer.reset();
      SmartDashboard.putNumber("TagID", tagID);
      Pose3d robotPose = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_4_NAME);

      double xSpeed = xController.calculate(-robotPose.getZ());
      double ySpeed = yController.calculate(robotPose.getX()); 
      double rotValue = rotationController.calculate(robotPose.getRotation().getY());
      SmartDashboard.putNumber("targetX", robotPose.getZ());
      SmartDashboard.putNumber("targetY", robotPose.getX());
      SmartDashboard.putNumber("targetRot", robotPose.getRotation().getY());
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