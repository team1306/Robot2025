package frc.robot.dashboardv2.networktables;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.dashboardv2.entry.ConfigOptions;

import java.util.HashSet;
import java.util.Set;

import static frc.robot.dashboardv2.networktables.DashboardUtil.ensureCorrectConfigOptions;

/**
 * Mappings to transform fields to valid NetworkTables type. See {@link #doubleMapping} or {@link #rotationMapping} for an example on how to write a mapping
 */
public class DashboardMappings {

    public static Mapping<Double, Double> doubleMapping =
            new Mapping<>(double.class, double.class,
                    (value, options) -> value, (value, options) -> value);

    public static Mapping<Integer, Double> integerMapping =
            new Mapping<>(int.class, double.class,
                    (value, options) -> value.doubleValue(), (value, options) -> value.intValue());

    public static Mapping<Float, Double> floatMapping =
            new Mapping<>(float.class, double.class,
                    (value, options) -> value.doubleValue(), (value, options) -> value.floatValue());

    public static Mapping<String, String> stringMapping =
            new Mapping<>(String.class, String.class,
                    (value, options) -> value, (value, options) -> value);

    public static Mapping<Boolean, Boolean> booleanMapping =
            new Mapping<>(boolean.class, boolean.class,
                    (value, options) -> value, (value, options) -> value);

    public static Mapping<double[], double[]> doubleArrayMapping =
            new Mapping<>(double[].class, double[].class,
                    (value, options) -> value, (value, options) -> value);
    
    public static Mapping<Rotation2d, Double> rotationMapping =
            new Mapping<>(Rotation2d.class, double.class,
                    (value, options) -> {
                        ensureCorrectConfigOptions(Rotation2d.class.getName(), options, ConfigOptions.InRadians, ConfigOptions.InDegrees);

                        if (options == ConfigOptions.InDegrees) {
                            return value.getDegrees();
                        }
                        return value.getRadians();
                    },
                    (value, options) -> {
                        ensureCorrectConfigOptions(Rotation2d.class.getName(), options, ConfigOptions.InRadians, ConfigOptions.InDegrees);

                        if (options == ConfigOptions.InDegrees) {
                            return Rotation2d.fromDegrees(value);
                        }
                        return Rotation2d.fromRadians(value);
                    });

    public static Set<Mapping<?, ?>> mappings = new HashSet<>();

    static {
        mappings.add(doubleMapping);
        mappings.add(stringMapping);
        mappings.add(integerMapping);
        mappings.add(floatMapping);
        mappings.add(booleanMapping);
        mappings.add(rotationMapping);

        mappings.add(doubleArrayMapping);
    }

    /**
     * Find a mapping from the list of mappings given type
     * @param type the input mapping type
     * @return the mapping with the input type the same as parameter
     */
    public static Mapping<?, ?> findMapping(Class<?> type) {
        for (Mapping<?, ?> mapping : mappings) {
            if (mapping.matches(type)) return mapping;
        }
        throw new IllegalArgumentException("No mapping for " + type);
    }

    @FunctionalInterface
    public interface MappingFunction<Input, ConfigOptions, Output> {
        Output apply(Input input, ConfigOptions options);
    }

    public record Mapping<Input, Output>
            (Class<Input> inputType, Class<Output> outputType,
             MappingFunction<Input, ConfigOptions, Output> inputMapper,
             MappingFunction<Output, ConfigOptions, Input> outputMapper) {
        
        public boolean matches(Class<?> type) {
            return type.isAssignableFrom(this.inputType);
        }
    }
}
