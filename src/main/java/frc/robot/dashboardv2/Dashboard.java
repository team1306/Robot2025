package frc.robot.dashboardv2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.dashboardv2.entry.CachedEntry;
import frc.robot.dashboardv2.entry.Entry;
import frc.robot.dashboardv2.networktables.DashboardEntry;
import frc.robot.dashboardv2.networktables.DashboardPublisher;
import frc.robot.dashboardv2.networktables.DashboardSendable;
import frc.robot.dashboardv2.networktables.DashboardSubscriber;

import java.util.*;
import java.util.stream.Collectors;

public class Dashboard {
    public static final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(DashboardConfig.dashboardTable);
    private static final Set<CachedEntry> entries = new HashSet<>();
    private static final Map<CachedEntry, DashboardEntry> ntEntryMap = new HashMap<>();

    /**
     * Add a class to the Dashboard to be updated from NetworkTables. This does not directly add any fields to be published/subscribed to from NetworkTables.
     * Use {@link Entry} and annotate fields with it. 
     * @see Entry
     * @param object the class object. This usually is called in a constructor with (this) as a parameter
     */
    public static void addDashboardClass(Object object) {
        entries.addAll(
                Arrays.stream(object.getClass().getDeclaredFields())
                        .filter(field -> field.isAnnotationPresent(Entry.class))
                        .map(field -> new CachedEntry(field, field.getAnnotation(Entry.class), object))
                        .toList()
        );
        ntEntryMap.putAll(entries.stream().collect(Collectors.toMap(entry -> entry, entry ->
                switch (entry.entry().type()) {
                    case Publisher -> new DashboardPublisher<>(entry, entry.getFieldType());
                    case Subscriber -> new DashboardSubscriber<>(entry, entry.getFieldType());
                    case Sendable -> new DashboardSendable(entry);
                }
        )));
    }

    /**
     * Update all the entries put to Network tables. This method should be called in {@link Robot#robotPeriodic()} 
     */
    public static void update() {
        ntEntryMap.values().forEach(DashboardEntry::update);
    }

    public static class DashboardConfig {
        public static final String dashboardTable = "Dashboard";
    }
}
