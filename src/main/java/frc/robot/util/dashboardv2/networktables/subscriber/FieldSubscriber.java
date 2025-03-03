package frc.robot.util.dashboardv2.networktables.subscriber;

import frc.robot.util.dashboardv2.entry.CachedEntry;
import frc.robot.util.dashboardv2.networktables.DashboardEntry;
import frc.robot.util.dashboardv2.networktables.DashboardUtil;

public class FieldSubscriber<Input, Output> extends DashboardSubscriber<Input, Output> implements DashboardEntry {

    private final CachedEntry entry;
    
    @SuppressWarnings("unchecked")
    public FieldSubscriber(CachedEntry entry, Class<Input> inputType) {
        super(entry.getKey(), () -> (Input) entry.getFieldValue(), inputType, entry.entry().config());

        this.entry = entry;
        DashboardUtil.ensureFieldInitialized(entry.field().getName(), entry.getFieldValue());
    }

    @Override
    public void update() {
        entry.setFieldValue(super.retrieveNetworkTablesValue());
    }
}
