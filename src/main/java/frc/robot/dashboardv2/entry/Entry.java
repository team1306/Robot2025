package frc.robot.dashboardv2.entry;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target({ElementType.FIELD, ElementType.METHOD})
public @interface Entry {
    /**
     * The key used for the entry in NetworkTables
     * @return the string used for NetworkTables. Defaults to a table with the name of the class and name of field 
     */
    String key() default "";

    /**
     * Type of NetworkTable Entry. (Publisher, Subscriber, Sendable)
     * @return the type
     */
    EntryType type();

    /**
     * Any config options needed for transforming a field into a valid NetworkTables type. 
     * @see frc.robot.dashboardv2.networktables.DashboardMappings
     * @return any config options. Defaults to none
     */
    ConfigOptions config() default ConfigOptions.NONE;
}

