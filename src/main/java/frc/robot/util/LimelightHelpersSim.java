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
  
   
    public static double[] getBotPose_TargetSpace(SwerveSubsystem drivebase, Pose2d tagPose) {
        // 1) get robot in field
        Pose2d robotField = drivebase.getPose();
        // 2) compute robot relative to tag:  tag ↦ field, so invert tag to go field ↦ tag
        Pose2d robotRelative = robotField.relativeTo(tagPose);
        
        // 3) push out for your auto-align (if desired)
        SmartDashboard.putNumber("targetXAutoAlign", robotRelative.getX());
        SmartDashboard.putNumber("targetYAutoAlign", robotRelative.getY());
        SmartDashboard.putNumber("targetYawAutoAlign", robotRelative.getRotation().getDegrees());
        
        // 4) return as array [x, y, yaw] – you can expand to 6-element if you really need z/pitch/roll
        return new double[] {
          robotRelative.getX(),
          0,
          robotRelative.getY(),
          0,
          robotRelative.getRotation().getDegrees(),
          0,
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
