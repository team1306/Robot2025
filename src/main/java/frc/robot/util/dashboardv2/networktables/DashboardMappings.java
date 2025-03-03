package frc.robot.util.dashboardv2.networktables;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.dashboardv2.entry.ConfigOptions;

import java.util.HashSet;
import java.util.Set;

import static frc.robot.util.dashboardv2.networktables.DashboardUtil.ensureCorrectConfigOptions;

/**
 * Mappings to transform fields to valid NetworkTables type. See {@link #double1Mapping} or {@link #rotationMapping} for an example on how to write a mapping
 */
public class DashboardMappings {

    public static Mapping<Double, Double> double1Mapping =
            new Mapping<>(double.class, double.class,
                    (value, options) -> value, (value, options) -> value);

    public static Mapping<Double, Double> double2Mapping =
            new Mapping<>(Double.class, double.class,
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

    public static Mapping<Boolean, Boolean> boolean1Mapping =
            new Mapping<>(boolean.class, boolean.class,
                    (value, options) -> value, (value, options) -> value);

    public static Mapping<Boolean, Boolean> boolean2Mapping =
            new Mapping<>(Boolean.class, boolean.class,
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
        mappings.add(double1Mapping);
        mappings.add(double2Mapping);

        mappings.add(stringMapping);
        mappings.add(integerMapping);
        mappings.add(floatMapping);
        
        mappings.add(boolean1Mapping);
        mappings.add(boolean2Mapping);

        mappings.add(rotationMapping);

        mappings.add(doubleArrayMapping);
    }

    /**
     * Add a mapping to the list of mappings
     * Must be done in a static constructor, since the mappings are used immediately at runtime
     * @param mapping the mapping to add
     */
    public static void addMapping(Mapping<?, ?> mapping) {
        mappings.add(mapping);
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
