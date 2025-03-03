package frc.robot.util.dashboardv2.entry;

public enum ConfigOptions {
    NONE,
    InDegrees,
    InRadians,
    InBaseUnit,
    CONFIG_0,
    CONFIG_1,
    CONFIG_2,
    CONFIG_3,
    CONFIG_4,
    CONFIG_5,
    CONFIG_6,
    CONFIG_7,
    CONFIG_8,
    CONFIG_9;
    
    public ConfigOptions getConfigOption(int option) {
        if(option < 0 || option >= values().length) throw new IndexOutOfBoundsException("Invalid option: " + option);
        return ConfigOptions.valueOf("CONFIG_" + option);
    }
}
