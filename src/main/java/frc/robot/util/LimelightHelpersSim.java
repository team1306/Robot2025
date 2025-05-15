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
  
   
    public static double[] getBotPose_TargetSpace(SwerveSubsystem drivebase){
        Pose2d pose = drivebase.getPose();
        double[] poseArray = {-1-pose.getY(), 0, -1-pose.getX(), 0, 90-pose.getRotation().getDegrees(), 0};
         SmartDashboard.putNumber("targetYAutoAlign", -1-pose.getY());
         SmartDashboard.putNumber("targetXAutoAlign", -1-pose.getX());
        return poseArray;
    }
    public static boolean getTV(){
        return true;
    }
    public static double getFiducialID(){
        return 21;
    }

}
