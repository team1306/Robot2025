package frc.robot.commands.autos;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldLocation {
    public static Pose2d A = new Pose2d(5.737, 4.186, Rotation2d.k180deg);
    public static Pose2d B = new Pose2d(5.713, 3.863, Rotation2d.k180deg);
    public static Pose2d C = new Pose2d(5.253, 3.041, Rotation2d.fromDegrees(120));
    public static Pose2d D = new Pose2d(4.965, 2.882, Rotation2d.fromDegrees(120));
    public static Pose2d E = new Pose2d(4.020, 2.883, Rotation2d.fromDegrees(60));
    public static Pose2d F = new Pose2d(3.720, 3.045, Rotation2d.fromDegrees(60)); 
    public static Pose2d G = new Pose2d(3.251, 3.859, Rotation2d.kZero);
    public static Pose2d H = new Pose2d(3.251, 4.186, Rotation2d.kZero);
    public static Pose2d I = new Pose2d(3.724, 5.024, Rotation2d.fromDegrees(-60)); 
    public static Pose2d J = new Pose2d(4.007, 5.188, Rotation2d.fromDegrees(-60));
    public static Pose2d K = new Pose2d(4.967, 5.183, Rotation2d.fromDegrees(-120));
    public static Pose2d L = new Pose2d(5.254, 5.020, Rotation2d.fromDegrees(-120)); 


    public static Pose2d CORAL_STATION_LEFT      =new Pose2d(0, 0, Rotation2d.fromDegrees(306));
    public static Pose2d CORAL_STATION_RIGHT     =new Pose2d(0, 0, Rotation2d.fromDegrees(54));


    private static Translation2d reefCenter = new Translation2d(Inches.of(176.183045), Inches.of(158.750907));
    private static Distance reefEdgeDistance = Inches.of(36.808);
    private static Distance robotWidth = Inches.of(14.625);
    private static double reefOffset = 6.469;

    static{
        
    }
}
