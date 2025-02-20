package frc.robot.dashboardv2.networktables;

import edu.wpi.first.networktables.GenericPublisher;
import frc.robot.dashboardv2.Dashboard;
import frc.robot.dashboardv2.entry.CachedEntry;

import static frc.robot.dashboardv2.networktables.DashboardUtil.getStringFromClass;

public class DashboardPublisher<Input, Output> implements DashboardEntry {

    private final CachedEntry entry;

    private final DashboardMappings.Mapping<Input, Output> mapping;

    private final GenericPublisher publisher;

    @SuppressWarnings("unchecked")
    public DashboardPublisher(CachedEntry entry, Class<Input> type) {
        this.entry = entry;
        this.mapping = (DashboardMappings.Mapping<Input, Output>) DashboardMappings.findMapping(type);

        this.publisher = Dashboard.networkTable.getEntry(entry.getKey()).getTopic().genericPublish(getStringFromClass(mapping.outputType()));

        DashboardUtil.ensureFieldInitialized(entry.field().getName(), entry.getFieldValue());
    }

    @SuppressWarnings("unchecked")
    @Override
    public void update() {
        publisher.setValue(mapping.inputMapper().apply((Input) entry.getFieldValue(), entry.entry().config()));
    }
}
