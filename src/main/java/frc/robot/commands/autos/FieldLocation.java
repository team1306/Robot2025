package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum FieldLocation {

    REEF_CLOSEST_LEFT       (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(0))),
    REEF_CLOSEST_RIGHT      (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(0))),
    REEF_FARTHEST_LEFT      (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(180))),
    REEF_FARTHEST_RIGHT     (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(180))),
    REEF_CLOSE_LEFT_LEFT    (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(-60))),
    REEF_CLOSE_LEFT_RIGHT   (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(-60))),
    REEF_FAR_LEFT_LEFT      (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(-120))),
    REEF_FAR_LEFT_RIGHT     (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(-120))),
    REEF_CLOSE_RIGHT_LEFT   (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(60))),
    REEF_CLOSE_RIGHT_RIGHT  (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(60))),
    REEF_FAR_RIGHT_LEFT     (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(120))),
    REEF_FAR_RIGHT_RIGHT    (new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.fromDegrees(120)));


    private final Pose2d pose;
    private FieldLocation(Pose2d pose) {
        this.pose = pose;
    }

    public Pose2d getPose() {
        System.out.println("Values for fieldLocation not set.");
        return pose;
    }
}
