package frc.robot.util.dashboardv2.networktables;

import frc.robot.util.dashboardv2.entry.ConfigOptions;

import java.util.Arrays;

public class DashboardUtil {

    public static void ensureFieldInitialized(String fieldName, Object value) {
        if (value == null)
            throw new NullPointerException(fieldName + " cannot be null\nEnsure it is initialized properly");
    }

    public static void ensureCorrectConfigOptions(String typeName, ConfigOptions option, ConfigOptions... validOptions) {
        if (!Arrays.asList(validOptions).contains(option))
            throw new IllegalArgumentException("Invalid config options for type " + typeName + "\n" + option + " is not " + Arrays.asList(validOptions));
    }

    /**
     * Gets string from generic class
     *
     * @param data the data to check
     * @return type string of the data, or empty string if no match
     */
    public static String getStringFromClass(Class<?> data) {
        if (data.isAssignableFrom(boolean.class)) {
            return "boolean";
        } else if (data.isAssignableFrom(float.class)) {
            return "float";
        } else if (data.isAssignableFrom(long.class)) {
            return "int";
        } else if (data.isAssignableFrom(double.class) || data.isAssignableFrom(int.class)) {
            return "double";
        } else if (data.isAssignableFrom(String.class)) {
            return "string";
        } else if (data.isAssignableFrom(boolean[].class)) {
            return "boolean[]";
        } else if (data.isAssignableFrom(float[].class)) {
            return "float[]";
        } else if (data.isAssignableFrom(long[].class)) {
            return "int[]";
        } else if (data.isAssignableFrom(double[].class)) {
            return "double[]";
        } else if (data.isAssignableFrom(String[].class)) {
            return "string[]";
        } else if (data.isAssignableFrom(byte[].class)) {
            return "raw";
        }
        throw new IllegalArgumentException("Illegal type to put to NetworkTables: " + data.getTypeName());
    }
}
