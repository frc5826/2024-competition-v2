package frc.robot.led;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;
import java.util.List;

public class TeensyLED {
    private final ArrayList<LED> leds;
    private final int LEDCount;
    private final CAN device;

    public TeensyLED(int deviceID, int LEDCount, Color initialColor) {
        device = new CAN(deviceID);
        this.LEDCount = LEDCount;
        leds = new ArrayList<>(LEDCount);
        for(int i = 0; i < LEDCount; i++) {
            leds.add(new LED(i, this, initialColor));
            setLED(i, initialColor);
        }
    }

    public void setLED(int index, Color color) {
        if(index > LEDCount) {
            System.err.println("LED " + index + " is out of bounds for the controller!!");
            return;
        }
        device.writePacket(new byte[] {
                        (byte) (color.red*255),
                        (byte) (color.blue * 255),
                        (byte) (color.green*255),
                        (byte) index}
                , 10);
    }

    public void setAll(Color color) {
        for(int i = 0; i < LEDCount; i++)
        device.writePacket(new byte[] {
                        (byte) (color.red*255),
                        (byte) (color.blue*255),
                        (byte) (color.green*255),
                        (byte) i}
                , 10);
    }

    public List<LED> getLEDs() {
        return leds;
    }
}
