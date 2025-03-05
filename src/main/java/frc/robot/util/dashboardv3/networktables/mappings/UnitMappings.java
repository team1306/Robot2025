package frc.robot.util.dashboardv3.networktables.mappings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.util.dashboardv3.networktables.mappings.UnitMappings.DistanceConfiguration.INCHES;
import static frc.robot.util.dashboardv3.networktables.mappings.UnitMappings.DistanceConfiguration.METERS;
import static frc.robot.util.dashboardv3.networktables.mappings.UnitMappings.RotationConfiguration.RADIANS;
import static frc.robot.util.dashboardv3.networktables.mappings.UnitMappings.RotationConfiguration.ROTATIONS;

public class UnitMappings {
    @MappingType
    public static Mapping<Distance, Double> distanceMapping = new Mapping<>(Distance.class, double.class, NetworkTableType.kDouble) {
        @Override
        public Double toNT(Distance fieldValue, String config) {
            if(config == null) config = "";

            return switch (config) {
                case INCHES -> fieldValue.in(Inches);
                case METERS -> fieldValue.in(Meters);
                default -> fieldValue.baseUnitMagnitude();
            };
        }

        @Override
        public Distance toField(Double ntValue, String config) {
            if(config == null) config = "";

            return switch (config) {
                case INCHES -> Inches.of(ntValue);
                case METERS -> Meters.of(ntValue);
                default -> Distance.ofBaseUnits(ntValue, BaseUnits.DistanceUnit);
            };
        }
    };
    
    public static class DistanceConfiguration{
        public static final String INCHES = "inches";
        public static final String METERS = "meters";
    }
    
    @MappingType
    public static Mapping<Rotation2d, Double> rotation2dDoubleMapping = new Mapping<>(Rotation2d.class, double.class, NetworkTableType.kDouble) {

        @Override
        public Double toNT(Rotation2d fieldValue, String config) {
            if(config == null) config = "";
            return switch (config) {
                case ROTATIONS -> fieldValue.getRotations();
                case RADIANS -> fieldValue.getRadians();
                default -> fieldValue.getDegrees();
            };
        }

        @Override
        public Rotation2d toField(Double ntValue, String config) {
            if(config == null) config = "";

            return switch (config) {
                case ROTATIONS -> Rotation2d.fromRotations(ntValue);
                case RADIANS -> Rotation2d.fromRadians(ntValue);
                default -> Rotation2d.fromDegrees(ntValue);
            };
        }
    };


    public static class RotationConfiguration{
        public static final String DEGREES = "degrees";
        public static final String RADIANS = "radians";
        public static final String ROTATIONS = "rotations";
    }
}
