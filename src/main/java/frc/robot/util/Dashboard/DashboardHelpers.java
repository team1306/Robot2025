package frc.robot.util.Dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import lombok.SneakyThrows;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * This class is a set of tools to write less code for SmartDashboard related methods. It primarily uses annotations to achieve this.
 */
public class DashboardHelpers {
    private static final Set<GetField> getFieldsToUpdate = new HashSet<>();
    private static final Set<PutField> putFieldsToUpdate = new HashSet<>();
    private static final Set<PutMethod> putMethodsToUpdate = new HashSet<>();

    /**
     * Adds a class to check for any {@link GetValue} or {@link PutValue} annotations.
     * This will update the fields with {@link GetValue} from the value on SmartDashboard.
     * This will put the value of any fields or call any methods with {@link PutValue} to SmartDashboard
     *
     * @param instance the instance of the class to check
     */
    public static void addUpdateClass(Object instance) {
        Arrays.stream(instance.getClass().getDeclaredFields())
                .filter(field -> field.isAnnotationPresent(GetValue.class)).forEach(field -> {
                    field.setAccessible(true);
                    String key = field.getAnnotation(GetValue.class).key();
                    if (key.isBlank())
                        key = field.getName();
                    key = instance.getClass().getSimpleName() + "/" + key;
                    Object value = getFieldValue(instance, field);
                    if (!SmartDashboard.getEntry(key).exists())
                        putValue(key, value);
                    getFieldsToUpdate.add(new GetField(instance, field, key, value));
                });

        Arrays.stream(instance.getClass().getDeclaredFields())
                .filter(field -> field.isAnnotationPresent(PutValue.class))
                .forEach(field -> {
                    field.setAccessible(true);
                    String key = field.getAnnotation(PutValue.class).key();
                    if (key.isBlank())
                        key = field.getName();
                    key = instance.getClass().getSimpleName() + "/" + key;
                    putFieldsToUpdate.add(new PutField(instance, field, key));
                });

        Arrays.stream(instance.getClass().getDeclaredMethods())
                .filter(method -> method.isAnnotationPresent(PutValue.class))
                .forEach(method -> {
                    method.setAccessible(true);
                    String key = method.getAnnotation(PutValue.class).key();
                    if (key.isBlank())
                        key = method.getName();
                    key = instance.getClass().getSimpleName() + "/" + key;
                    putMethodsToUpdate.add(new PutMethod(instance, method, key));
                });

    }

    /**
     * Updates all the fields and methods in the registered classes by either putting or getting a specific value.
     * This method should be called only once in {@link Robot#robotPeriodic()}
     */
    public static void update() {
        getFieldsToUpdate.forEach(DashboardHelpers::handleGetTypes);
        putFieldsToUpdate.forEach(DashboardHelpers::internalPutValue);
        putMethodsToUpdate.forEach(DashboardHelpers::internalPutValue);
    }

    /**
     * Gets the current value of a field
     *
     * @param instance the instance of the class with the field in it
     * @param field    the field to get from
     * @return the value of the field
     */
    @SneakyThrows(IllegalAccessException.class)
    private static Object getFieldValue(Object instance, Field field) {
        return field.get(instance);
    }

    /**
     * Gets the value of a method
     *
     * @param instance the instance of the class with the method in it
     * @param method   the method to get the value from
     * @return the value of the method
     */
    @SneakyThrows({IllegalAccessException.class, InvocationTargetException.class})
    private static Object getMethodValue(Object instance, Method method) {
        return method.invoke(instance);
    }

    @SneakyThrows(IllegalAccessException.class)
    private static void handleGetTypes(GetField getField) {
        Field field = getField.field;
        Class<?> type = field.getType();

        if (type.equals(double.class)) {
            field.set(getField.instance, SmartDashboard.getNumber(getField.key, (double) getField.defaultValue));
        } else if (type.equals(String.class)) {
            String stringDefault = (String) getField.defaultValue;
            field.set(getField.instance, SmartDashboard.getString(getField.key, stringDefault == null ? "" : stringDefault));
        } else if (type.equals(boolean.class)) {
            field.set(getField.instance, SmartDashboard.getBoolean(getField.key, (boolean) getField.defaultValue));
        } else {
            throw new RuntimeException("Unsupported getValue type: " + type);
        }
    }

    @SneakyThrows(IllegalAccessException.class)
    private static void internalPutValue(PutField putField) {
        putValue(putField.key, putField.field.get(putField.instance));
    }

    private static void internalPutValue(PutMethod putField) {
        putValue(putField.key, getMethodValue(putField.instance, putField.method));
    }

    /**
     * Handles the putting of an {@link Object} to SmartDashboard without explicitly writing the type
     *
     * @param key   the key to put to SmartDashboard
     * @param value the value to put to SmartDashboard. This must be one of the valid types ({@link Double}, {@link String}, {@link Boolean})
     */
    public static void putValue(String key, Object value) {
        if (value instanceof Double) {
            SmartDashboard.putNumber(key, (double) value);
        } else if (value instanceof String) {
            SmartDashboard.putString(key, (String) value);
        } else if (value instanceof Boolean) {
            SmartDashboard.putBoolean(key, (boolean) value);
        } else {
            throw new RuntimeException("Unsupported putValue type: " + (value == null ? "void is not a valid type" : value.getClass()));
        }
    }

    private record GetField(Object instance, Field field, String key, Object defaultValue) {
    }

    private record PutField(Object instance, Field field, String key) {
    }

    private record PutMethod(Object instance, Method method, String key) {
    }

}
