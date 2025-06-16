package frc.robot.util;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

public class LimelightHelpersSim {
  
   
    public static double[] getTargetPose_RobotSpace(SwerveSubsystem drivebase, Pose2d tagPose) {
   
        Pose2d robotPose = drivebase.getPose();
        
        Pose2d robotRelative = robotPose.relativeTo(tagPose);
  
        SmartDashboard.putNumber("targetXAutoAlign", robotRelative.getX());
        SmartDashboard.putNumber("targetYAutoAlign", robotRelative.getY());
        SmartDashboard.putNumber("targetYawAutoAlign", robotRelative.getRotation().getDegrees());

        return new double[] {
          robotRelative.getX(),
          robotRelative.getY(),
          0,
          0,
          robotRelative.getRotation().getDegrees(),
          0
        };
    }
    public static boolean getTV(){
        return true;
    }
    public static double getFiducialID(){
        return 21;
    }

}
