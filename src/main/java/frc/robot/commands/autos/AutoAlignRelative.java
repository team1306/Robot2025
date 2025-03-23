package frc.robot.commands.autos;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.LimelightHelpers;
import frc.robot.Constants;

public class AutoAlignRelative {
    
    public static final Set<Integer> REEF_IDS = new HashSet<>(Arrays.asList(22, 21, 19, 18, 17, 6, 7, 8, 9, 10, 11));
    public static enum ReefSide{
        LEFT,
        RIGHT
    }

    @SuppressWarnings("unlikely-arg-type")
    public static Pose2d getReefOffsetFromApriltag(ReefSide reefSide){
        if (REEF_IDS.contains(LimelightHelpers.getFiducialID(Constants.LIMELIGHT_4_NAME))) return Pose2d.kZero;
        Pose3d fiducialPos = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_4_NAME);

        return switch(reefSide){
            case LEFT -> new Pose2d(fiducialPos.getMeasureX().minus(FieldLocation.reefOffset), fiducialPos.getMeasureZ(), fiducialPos.getRotation().toRotation2d());
            case RIGHT -> new Pose2d(fiducialPos.getMeasureX().plus(FieldLocation.reefOffset), fiducialPos.getMeasureZ(), fiducialPos.getRotation().toRotation2d());
        };   
    }
}


