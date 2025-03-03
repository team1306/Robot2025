package frc.robot.util.dashboardv2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.dashboardv2.entry.CachedEntry;
import frc.robot.util.dashboardv2.entry.Entry;
import frc.robot.util.dashboardv2.networktables.DashboardEntry;
import frc.robot.util.dashboardv2.networktables.DashboardSendable;
import frc.robot.util.dashboardv2.networktables.publisher.DashboardPublisher;
import frc.robot.util.dashboardv2.networktables.publisher.FieldPublisher;
import frc.robot.util.dashboardv2.networktables.subscriber.DashboardSubscriber;
import frc.robot.util.dashboardv2.networktables.subscriber.FieldSubscriber;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public class Dashboard {
    public static final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(DashboardConfig.dashboardTable);
    private static final Set<CachedEntry> entries = new HashSet<>();
    private static final HashMap<String, DashboardEntry> ntEntries = new HashMap<>();

    /**
     * Add a class to the Dashboard to be updated from NetworkTables. This does not directly add any fields to be published/subscribed to from NetworkTables.
     * Use {@link Entry} and annotate fields with it.
     *
     * @param object the class object. This usually is called in a constructor with (this) as a parameter
     * @see Entry
     */
    public static void addDashboardClass(Object object) {
        entries.addAll(
                Arrays.stream(object.getClass().getDeclaredFields())
                        .filter(field -> field.isAnnotationPresent(Entry.class))
                        .map(field -> new CachedEntry(field, field.getAnnotation(Entry.class), object))
                        .toList()
        );
        ntEntries.putAll(
                entries.stream().collect(
                        Collectors.toMap(CachedEntry::getKey, entry ->
                                switch (entry.entry().type()) {
                                    case Publisher -> new FieldPublisher<>(entry, entry.getFieldType());
                                    case Subscriber -> new FieldSubscriber<>(entry, entry.getFieldType());
                                    case Sendable -> new DashboardSendable(entry);
                                }
                        )
                )
        );
    }

    /**
     * Create a trigger bound to a NetworkTables boolean value
     *
     * @param key the key for NetworkTables
     * @return the trigger bound to the NetworkTables value
     */
    public static Trigger createButton(String key) {
        return createButton(key, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Create a trigger bound to a NetworkTables boolean value
     *
     * @param key       the key for NetworkTables
     * @param eventLoop the eventloop to bind this trigger to
     * @return the trigger bound to the NetworkTables value and eventloop
     */
    public static Trigger createButton(String key, EventLoop eventLoop) {
        DashboardSubscriber<Boolean, Boolean> subscriber = new DashboardSubscriber<>(key, () -> false, boolean.class);

        return new Trigger(eventLoop, subscriber::retrieveNetworkTablesValue);
    }

    /**
     * Create a publisher and post a generic value to NetworkTables
     *
     * @param key   the NetworkTables key
     * @param value the value to publish
     */
    @SuppressWarnings("unchecked")
    public static <Type> void putValue(String key, Type value) {
        DashboardPublisher<Type, Object> publisher;
        if (ntEntries.containsKey(key)) {
            try {
                publisher = (DashboardPublisher<Type, Object>) ntEntries.get(key);
            } catch (ClassCastException e) {
                throw new ClassCastException(key + " is not the correct type publisher.");
            }
        } else {
            publisher = (DashboardPublisher<Type, Object>) new DashboardPublisher<>(key, value.getClass());
        }
        publisher.changeSupplier(value);

        ntEntries.put(key, publisher);
    }

    /**
     * Update all the entries put to Network tables. This method should be called in {@link Robot#robotPeriodic()}
     */
    public static void update() {
        ntEntries.values().forEach(DashboardEntry::update);
    }

    public static class DashboardConfig {
        public static final String dashboardTable = "Dashboard";
    }
}
