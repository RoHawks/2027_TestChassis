package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Lights {
    AddressableLED leds;
    AddressableLEDBuffer buffer;

    public Lights(int port, int length) {
        leds = new AddressableLED(port);

        buffer = new AddressableLEDBuffer(length);
        leds.setLength(length);

        leds.start();
    }

    public void display(Color color) {
        LEDPattern.solid(color).applyTo(buffer);

        leds.setData(buffer);
    }
    public void off() {
        display(Color.kBlack);
    }
}
