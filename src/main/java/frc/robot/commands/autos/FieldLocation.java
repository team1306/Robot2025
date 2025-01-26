package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldLocation {
    
    public static Pose2d A;
    public static Pose2d B;
    public static Pose2d C;
    public static Pose2d D;
    public static Pose2d E;
    public static Pose2d F; 
    public static Pose2d G;
    public static Pose2d H;
    public static Pose2d I; 
    public static Pose2d J;
    public static Pose2d K;
    public static Pose2d L; 

    public static List<Pose2d> reefLocations;
    public static List<Pose2d> coralStationLocations;

    public static Pose2d CORAL_STATION_LEFT      =new Pose2d(0, 0, Rotation2d.fromDegrees(306));
    public static Pose2d CORAL_STATION_RIGHT     =new Pose2d(0, 0, Rotation2d.fromDegrees(54));
    
    private static final Translation2d reefCenter = new Translation2d(Inches.of(176.277), Inches.of(158.801));
    private static final Distance reefEdgeDistance = Inches.of(32.746);
    private static final Distance robotWidth = Inches.of(35.75);
    private static final Distance robotOffsetFromReef = Inches.of(10);

    private static final Distance reefOffset = Inches.of(6.469);

    private static final Rotation2d rotation120 = Rotation2d.fromDegrees(120);
    private static final Rotation2d rotation60 = Rotation2d.fromDegrees(60);
    
    static{
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

        reefLocations = Arrays.asList(A, B, C, D, E, F, G, H, I, J, K, L);
        coralStationLocations = Arrays.asList(CORAL_STATION_LEFT, CORAL_STATION_RIGHT);
    }

    private static Pose2d calculateReefPosition(Rotation2d angle, boolean leftSide){
        return new Pose2d(calculateReefRelativePosition(angle, leftSide).plus(reefCenter), angle);
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
    
    
}
