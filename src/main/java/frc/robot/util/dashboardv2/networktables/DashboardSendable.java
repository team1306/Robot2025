package frc.robot.util.dashboardv2.networktables;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import frc.robot.util.dashboardv2.Dashboard;
import frc.robot.util.dashboardv2.entry.CachedEntry;

public class DashboardSendable implements DashboardEntry {

    public final Sendable sendable;

    public DashboardSendable(CachedEntry entry) {
        if (entry.getFieldType().isAssignableFrom(Sendable.class))
            throw new IllegalArgumentException("The field " + entry.getKey() + " should be of type sendable");
        sendable = (Sendable) entry.getFieldValue();

        NetworkTable dataTable = Dashboard.networkTable.getSubTable(entry.getKey());
        SendableBuilderImpl builder = new SendableBuilderImpl();
        builder.setTable(dataTable);
        SendableRegistry.publish(sendable, builder);
        builder.startListeners();
        
        dataTable.getEntry(".name").setString(entry.getKey());
    }

    @Override
    public void update() {
        SendableRegistry.update(sendable);
    }
}
