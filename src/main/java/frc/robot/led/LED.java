package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.TeensyLED;

public class LED {
    private final int index;
    private final TeensyLED controller;

    private Color color;

    protected LED(int index, TeensyLED controller, Color color) {
        this.color = color;
        this.index = index;
        this.controller = controller;
    }

    public void setColor(Color color) {
        this.color = color;
        controller.setLED(index, color);
    }
}
