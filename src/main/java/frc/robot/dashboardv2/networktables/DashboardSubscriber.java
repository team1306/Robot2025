package frc.robot.dashboardv2.networktables;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.dashboardv2.Dashboard;
import frc.robot.dashboardv2.entry.CachedEntry;

import static frc.robot.dashboardv2.networktables.DashboardUtil.getStringFromClass;


public class DashboardSubscriber<Input, Output> implements DashboardEntry {

    private final CachedEntry entry;

    private final DashboardMappings.Mapping<Input, Output> mapping;

    private final GenericEntry genericEntry;

    private boolean firstPublish = true;

    @SuppressWarnings("unchecked")
    public DashboardSubscriber(CachedEntry entry, Class<Input> inputType) {
        this.entry = entry;
        this.mapping = (DashboardMappings.Mapping<Input, Output>) DashboardMappings.findMapping(inputType);

        this.genericEntry = Dashboard.networkTable.getEntry(entry.getKey()).getTopic().getGenericEntry(getStringFromClass(mapping.outputType()));

        DashboardUtil.ensureFieldInitialized(entry.field().getName(), entry.getFieldValue());
    }

    @SuppressWarnings("unchecked")
    @Override
    public void update() {
        if (firstPublish) {
            genericEntry.setValue(mapping.inputMapper().apply((Input) entry.getFieldValue(), entry.entry().config()));
            firstPublish = false;
        }

        Input value = mapping.outputMapper().apply((Output) genericEntry.get().getValue(), entry.entry().config());
        if (value == null) value = (Input) entry.getFieldValue();
        entry.setFieldValue(value);
    }

}
