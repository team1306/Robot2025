package frc.robot.commands.autos;

import java.util.*;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

import static edu.wpi.first.units.Units.*;

/**
 * A collection of Pose2d field locations
 */
public class FieldLocation {
    
    private static final Translation2d reefCenter = new Translation2d(Inches.of(176.277), Inches.of(158.801));
    private static final Distance reefEdgeDistance = Inches.of(32.746);
    private static final Distance robotWidth = Inches.of(35.75);
    private static final Distance robotOffsetFromReef = Inches.of(2.5);

    public static final Distance reefOffset = Inches.of(6.469);
    
    private static final Distance coralStationOffset = Inches.of(-7);
    
    private static final Rotation2d rotation120 = Rotation2d.fromDegrees(120);
    private static final Rotation2d rotation60 = Rotation2d.fromDegrees(60);

    public static Pose2d A, B, C, D, E, F, G, H, I, J, K, L;

    public static Pose2d CORAL_STATION_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(306));
    public static Pose2d CORAL_STATION_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(54));
    
    public static HashMap<Pose2d, Boolean> reefLocations;
    public static List<Pose2d> coralStationLocations;

    public static final Set<Integer> REEF_APRILTAG_IDS = new HashSet<>(Arrays.asList(22, 21, 19, 18, 17, 6, 7, 8, 9, 10, 11));

    public enum ReefSide{
        LEFT,
        RIGHT
    }

    static {
        refreshReefPositions();
    }
    
    public static void recalculateFieldPositions(){
        refreshReefPositions();

        CORAL_STATION_LEFT = new Pose2d(0.843, 7.4, Rotation2d.fromDegrees(126));
        CORAL_STATION_RIGHT = new Pose2d(0.840, 0.664, Rotation2d.fromDegrees(234));
        
        CORAL_STATION_RIGHT = CORAL_STATION_RIGHT.transformBy(new Transform2d(-(coralStationOffset.in(Meters) + robotWidth.in(Meters)), Inches.of(2).in(Meter), Rotation2d.kZero));
        CORAL_STATION_LEFT = CORAL_STATION_LEFT.transformBy(new Transform2d(-(coralStationOffset.in(Meters) + robotWidth.in(Meters)), 0, Rotation2d.kZero));

        CORAL_STATION_LEFT = Utilities.isRedAlliance() ? flipToRedSide(CORAL_STATION_LEFT) : CORAL_STATION_LEFT;
        CORAL_STATION_RIGHT = Utilities.isRedAlliance() ? flipToRedSide(CORAL_STATION_RIGHT) : CORAL_STATION_RIGHT;
        
        coralStationLocations = Arrays.asList(CORAL_STATION_LEFT, CORAL_STATION_RIGHT);
    }
    
    private static void refreshReefPositions(){
        A = calculateReefPosition(Rotation2d.k180deg, true);
        B = calculateReefPosition(Rotation2d.k180deg, false);
        C = calculateReefPosition(rotation120, true);
        D = calculateReefPosition(rotation120, false);
        E = calculateReefPosition(rotation60, true);
        F = calculateReefPosition(rotation60, false);
        G = calculateReefPosition(Rotation2d.kZero, true);
        H = calculateReefPosition(Rotation2d.kZero, false);
        I = calculateReefPosition(rotation60.unaryMinus(), true);
        J = calculateReefPosition(rotation60.unaryMinus(), false);
        K = calculateReefPosition(rotation120.unaryMinus(), true);
        L = calculateReefPosition(rotation120.unaryMinus(), false);

        reefLocations = new HashMap<>(){{
            put(A, true);
            put(B, false);
            put(C, true);
            put(D, false);
            put(E, true);
            put(F, false);
            put(G, true);
            put(H, false);
            put(I, true);
            put(J, false);
            put(K, true);
            put(L, false);
        }};
    }

    public static Pose2d getReefOffsetFromAprilTag(ReefSide reefSide){
        if (REEF_APRILTAG_IDS.contains((int) LimelightHelpers.getFiducialID(Constants.LIMELIGHT_4_NAME))) return Pose2d.kZero;
        Pose3d fiducialPos = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT_4_NAME);

        return switch(reefSide){
            case LEFT -> new Pose2d(fiducialPos.getMeasureX().minus(reefOffset), fiducialPos.getMeasureZ(), fiducialPos.getRotation().toRotation2d());
            case RIGHT -> new Pose2d(fiducialPos.getMeasureX().plus(reefOffset), fiducialPos.getMeasureZ(), fiducialPos.getRotation().toRotation2d());
        };
    }

    private static Pose2d calculateReefPosition(Rotation2d angle, boolean leftSide){
        Pose2d reefPosition = new Pose2d(calculateReefRelativePosition(angle, leftSide).plus(reefCenter), angle);
        if(Utilities.isRedAlliance())
            reefPosition = flipToRedSide(reefPosition);
        return reefPosition;
    }
    
    private static Translation2d calculateReefRelativePosition(Rotation2d angle, boolean leftSide){
        Translation2d reefPosition;
        
        reefPosition = new Translation2d(calculateTotalRobotDistance(), reefOffset.times(leftSide ? 1 : -1));
        reefPosition = reefPosition.rotateBy(angle.minus(Rotation2d.k180deg));
        return reefPosition;
    }
    
    private static Distance calculateTotalRobotDistance(){
        return reefEdgeDistance.plus(robotWidth.div(2)).plus(robotOffsetFromReef);
    }
    
    private static Pose2d flipToRedSide(Pose2d pose) {
        if (FlippingUtil.symmetryType != FlippingUtil.FieldSymmetry.kMirrored) System.out.println("Symmetry type changed to mirrored");
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored;
        return FlippingUtil.flipFieldPose(pose);
    }
}
