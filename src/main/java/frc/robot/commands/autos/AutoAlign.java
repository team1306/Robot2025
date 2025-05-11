
package frc.robot.commands.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
  private PIDController xController, yController, rotationController;
  private boolean direction;
  private Timer noTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;

  public AutoAlign(boolean direction, SwerveSubsystem drivebase) {
    xController = new PIDController(0, 0.0, 0);  
    yController = new PIDController(0, 0.0, 0);  
    rotationController = new PIDController(0, 0, 0);  
    this.direction = direction; // 1 = right reef, 0 = left reef

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

      double[] postions = LimelightHelpers.getBotPose_TargetSpace(Constants.LIMELIGHT_4_NAME);

      double xSpeed = xController.calculate(postions[2]);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotationController.calculate(postions[4]);

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
      
    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
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