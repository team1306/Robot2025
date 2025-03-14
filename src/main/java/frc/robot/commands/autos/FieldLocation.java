package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.Utilities;

/**
 * A collection of Pose2d field locations
 */
public class FieldLocation {
    public static List<Pose2d> reefLocations;
    public static List<Pose2d> coralStationLocations;

    public static Pose2d CORAL_STATION_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(306));
    public static Pose2d CORAL_STATION_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(54));

    private static final Translation2d reefCenter = new Translation2d(Inches.of(176.277), Inches.of(158.801));
    private static final Distance reefEdgeDistance = Inches.of(32.746);
    private static final Distance robotWidth = Inches.of(35.75);
    private static final Distance robotOffsetFromReef = Inches.of(2.5);

    private static final Distance reefOffset = Inches.of(6.469);
    
    private static final Rotation2d rotation120 = Rotation2d.fromDegrees(120);
    private static final Rotation2d rotation60 = Rotation2d.fromDegrees(60);

    public static Pose2d A = calculateReefPosition(Rotation2d.k180deg, true);
    public static Pose2d B = calculateReefPosition(Rotation2d.k180deg, false);
    public static Pose2d C = calculateReefPosition(rotation120, true);
    public static Pose2d D = calculateReefPosition(rotation120, false);
    public static Pose2d E = calculateReefPosition(rotation60, true);
    public static Pose2d F = calculateReefPosition(rotation60, false);
    public static Pose2d G = calculateReefPosition(Rotation2d.kZero, true);
    public static Pose2d H = calculateReefPosition(Rotation2d.kZero, false);
    public static Pose2d I = calculateReefPosition(rotation60.unaryMinus(), true);
    public static Pose2d J = calculateReefPosition(rotation60.unaryMinus(), false);
    public static Pose2d K = calculateReefPosition(rotation120.unaryMinus(), true);
    public static Pose2d L = calculateReefPosition(rotation120.unaryMinus(), false);
    
    public static void calculateReefPositions(){
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

        A = Utilities.isRedAlliance() ? flipToRedSide(A) : A;
        B = Utilities.isRedAlliance() ? flipToRedSide(B) : B;
        C = Utilities.isRedAlliance() ? flipToRedSide(C) : C;
        D = Utilities.isRedAlliance() ? flipToRedSide(D) : D;
        E = Utilities.isRedAlliance() ? flipToRedSide(E) : E;
        F = Utilities.isRedAlliance() ? flipToRedSide(F) : F;
        G = Utilities.isRedAlliance() ? flipToRedSide(G) : G;
        H = Utilities.isRedAlliance() ? flipToRedSide(H) : H;
        I = Utilities.isRedAlliance() ? flipToRedSide(I) : I;
        J = Utilities.isRedAlliance() ? flipToRedSide(J) : J;
        K = Utilities.isRedAlliance() ? flipToRedSide(K) : K;
        L = Utilities.isRedAlliance() ? flipToRedSide(L) : L;

        reefLocations = Arrays.asList(A, B, C, D, E, F, G, H, I, J, K, L);

        CORAL_STATION_LEFT = new Pose2d(0, 0, Rotation2d.fromDegrees(306));
        CORAL_STATION_RIGHT = new Pose2d(0, 0, Rotation2d.fromDegrees(54));

        CORAL_STATION_LEFT = Utilities.isRedAlliance() ? flipToRedSide(CORAL_STATION_LEFT) : CORAL_STATION_LEFT;
        CORAL_STATION_RIGHT = Utilities.isRedAlliance() ? flipToRedSide(CORAL_STATION_RIGHT) : CORAL_STATION_RIGHT;
        
        coralStationLocations = Arrays.asList(CORAL_STATION_LEFT, CORAL_STATION_RIGHT);
    }

    private static Pose2d calculateReefPosition(Rotation2d angle, boolean leftSide){
        Pose2d reefPosition = new Pose2d(calculateReefRelativePosition(angle, leftSide).plus(reefCenter), angle);
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
        if (FlippingUtil.symmetryType != FlippingUtil.FieldSymmetry.kMirrored) System.out.println("Symetry type changed to mirrored");
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kMirrored;
        return FlippingUtil.flipFieldPose(pose);
    }
}
