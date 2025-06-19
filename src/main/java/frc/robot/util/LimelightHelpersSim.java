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
  
   
    public static Pose3d getTargetPose3d_RobotSpace(SwerveSubsystem drivebase, Pose2d tagPose) {
   
        Pose2d robotPose = drivebase.getPose();
        
        Pose2d robotRelative = tagPose.relativeTo(robotPose);
  
        SmartDashboard.putNumber("alignErrorX", robotRelative.getX());
        SmartDashboard.putNumber("alignErrorY", -robotRelative.getY());
    

        return new Pose3d(robotRelative.getX(), -robotRelative.getY(), 0d, new Rotation3d(robotRelative.getRotation()));
    }
    public static boolean getTV(){
        return true;
    }
    public static double getFiducialID(){
        return 21;
    }

}
