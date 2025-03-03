package frc.robot.util.dashboardv2.networktables.publisher;

import edu.wpi.first.networktables.GenericPublisher;
import frc.robot.util.dashboardv2.Dashboard;
import frc.robot.util.dashboardv2.entry.ConfigOptions;
import frc.robot.util.dashboardv2.networktables.DashboardEntry;
import frc.robot.util.dashboardv2.networktables.DashboardMappings;

import java.util.function.Supplier;

import static frc.robot.util.dashboardv2.networktables.DashboardUtil.getStringFromClass;

public class DashboardPublisher<Input, Output> implements DashboardEntry {
    
    private Supplier<Input> inputSupplier;
    private final ConfigOptions options;
    
    private final DashboardMappings.Mapping<Input, Output> mapping;
    private final GenericPublisher publisher;
    
    private boolean oneTimePublish = false;

    @SuppressWarnings("unchecked")
    public DashboardPublisher(String key, Supplier<Input> supplier, Class<Input> type, ConfigOptions options) {
        this.mapping = (DashboardMappings.Mapping<Input, Output>) DashboardMappings.findMapping(type);
    
        this.inputSupplier = supplier;
        this.options = options;
        
        this.publisher = Dashboard.networkTable
                .getEntry(key)
                .getTopic()
                .genericPublish(getStringFromClass(mapping.outputType()));
    }
    
    public DashboardPublisher(String key, Supplier<Input> supplier, Class<Input> type) {
        this(key, supplier, type, ConfigOptions.NONE);
    }
    
    public DashboardPublisher(String key, Class<Input> type) {
        this(key, null, type, ConfigOptions.NONE);
    }

    @Override
    public void update() {
        if(inputSupplier == null) return;
        
        publisher.setValue(
                mapping.inputMapper().apply(inputSupplier.get(), options)
        );
        
        if(oneTimePublish){
            inputSupplier = null;
            oneTimePublish = false;
        }
    }
    
    public void changeSupplier(Input value) {
        inputSupplier = () -> value;
        oneTimePublish = true;
    }
}
