package frc.robot.dashboardv2.entry;

import lombok.SneakyThrows;

import java.lang.reflect.Field;

/**
 * Internal class used by {@link frc.robot.dashboardv2.Dashboard}
 */
public record CachedEntry(Field field, Entry entry, Object classObject) {
    public Class<?> getFieldType() {
        return field.getType();
    }

    public String getKey() {
        if (entry.key().isEmpty())
            return classObject.getClass().getSimpleName() + "/" + field.getName();

        return entry.key();
    }

    @SneakyThrows(IllegalAccessException.class)
    public <T> T getFieldValue(Class<T> type) {
        field.setAccessible(true);
        return type.cast(field.get(classObject));
    }

    @SneakyThrows(IllegalAccessException.class)
    public Object getFieldValue() {
        field.setAccessible(true);
        return field.get(classObject);
    }

    @SneakyThrows(IllegalAccessException.class)
    public void setFieldValue(Object value) {
        field.setAccessible(true);
        field.set(classObject, value);
    }
}
