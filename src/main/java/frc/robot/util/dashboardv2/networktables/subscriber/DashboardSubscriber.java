package frc.robot.util.dashboardv2.networktables.subscriber;

import edu.wpi.first.networktables.GenericEntry;
import frc.robot.util.dashboardv2.Dashboard;
import frc.robot.util.dashboardv2.entry.ConfigOptions;
import frc.robot.util.dashboardv2.networktables.DashboardMappings;

import java.util.function.Supplier;

import static frc.robot.util.dashboardv2.networktables.DashboardUtil.getStringFromClass;


public class DashboardSubscriber<Input, Output>{
    
    private final Supplier<Input> initialValueSupplier;
    private final ConfigOptions options;
    
    private final DashboardMappings.Mapping<Input, Output> mapping;
    private final GenericEntry genericEntry;

    private boolean firstPublish = true;

    @SuppressWarnings("unchecked")
    public DashboardSubscriber(String key, Supplier<Input> initialValueSupplier, Class<Input> inputType, ConfigOptions options) {
        this.mapping = (DashboardMappings.Mapping<Input, Output>) DashboardMappings.findMapping(inputType);

        this.initialValueSupplier = initialValueSupplier;
        this.options = options;
        
        this.genericEntry = 
                Dashboard.networkTable
                .getEntry(key)
                .getTopic()
                .getGenericEntry(getStringFromClass(mapping.outputType()));
    }
    
    public DashboardSubscriber(String key, Supplier<Input> initialValueSupplier, Class<Input> inputType) {
        this(key, initialValueSupplier, inputType, ConfigOptions.NONE);
    }

    @SuppressWarnings("unchecked")
    public Input retrieveNetworkTablesValue() {
        if (firstPublish) {
            genericEntry.setValue(
                    mapping.inputMapper()
                            .apply(initialValueSupplier.get(), options)
            );
            firstPublish = false;
        }

        Input value = mapping.outputMapper()
                .apply((Output) genericEntry.get().getValue(), options);
        
        if (value == null) value = initialValueSupplier.get();
        return value;
    }

}
