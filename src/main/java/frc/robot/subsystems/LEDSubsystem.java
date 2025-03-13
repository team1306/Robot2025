package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDSubsystem extends SubsystemBase {
    private final int kPort;
    private static final int kLength = 28;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 1 meter per second.


    public LEDSubsystem(int port) {
        kPort = port;
        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);
        m_led.setLength(kLength);
        m_led.start();


        // Set the default command to turn the strip off, otherwise the last colors written by
        // the last command to run will continue to be displayed.
        // Note: Other default patterns could be used instead!
        setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }


    @Override
    public void periodic() {
        // Periodically send the latest LED color data to the LED strip for it to display
        m_led.setData(m_buffer);
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_buffer));
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command setRed() {
        LEDPattern red = LEDPattern.solid(Color.kRed);
        return run(() -> red.applyTo(m_buffer));
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command setBlue() {
        LEDPattern blue = LEDPattern.solid(Color.kBlue);
        System.out.println("setblue");

        return run(() -> blue.applyTo(m_buffer));
    }

    public Command setAlliance() {
        boolean alliance = isRedAlliance();
        if (alliance) {
            LEDPattern color = LEDPattern.solid(Color.kRed);
            System.out.println("setred");

            return run(() -> color.applyTo(m_buffer));
        } else {
            LEDPattern color = LEDPattern.solid(Color.kBlue);
            System.out.println("setblue");

            return run(() -> color.applyTo(m_buffer));
        }

    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isEmpty() || (alliance.get() == DriverStation.Alliance.Red);
    }
}