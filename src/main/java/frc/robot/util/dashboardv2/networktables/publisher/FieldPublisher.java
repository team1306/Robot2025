package frc.robot.util.dashboardv2.networktables.publisher;

import frc.robot.util.dashboardv2.entry.CachedEntry;
import frc.robot.util.dashboardv2.networktables.DashboardUtil;

public class FieldPublisher<Input, Output> extends DashboardPublisher<Input, Output> {
    
    @SuppressWarnings("unchecked")
    public FieldPublisher(CachedEntry entry, Class<Input> type) {
        super(entry.getKey(), () -> (Input) entry.getFieldValue(), type, entry.entry().config());

        DashboardUtil.ensureFieldInitialized(entry.field().getName(), entry.getFieldValue());
    }
}
