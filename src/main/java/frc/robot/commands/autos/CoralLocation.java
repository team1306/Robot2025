package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;

public enum CoralLocation {

    CLOSEST_LEFT(true, Pose2d.kZero),
    CLOSEST_RIGHT(false, Pose2d.kZero),
    FARTHEST_LEFT(false, Pose2d.kZero),
    FARTHEST_RIGHT(false, Pose2d.kZero),
    CLOSE_LEFT_LEFT(false, Pose2d.kZero),
    CLOSE_LEFT_RIGHT(false, Pose2d.kZero),
    FAR_LEFT_LEFT(false, Pose2d.kZero),
    FAR_LEFT_RIGHT(false, Pose2d.kZero),
    CLOSE_RIGHT_LEFT(false, Pose2d.kZero),
    CLOSE_RIGHT_RIGHT(false, Pose2d.kZero),
    FAR_RIGHT_LEFT(false, Pose2d.kZero),
    FAR_RIGHT_RIGHT(false, Pose2d.kZero);


    boolean left;
    Pose2d pose;
    private CoralLocation(boolean left, Pose2d pose) {
        this.left = left;
        this.pose = pose;
    }
}
