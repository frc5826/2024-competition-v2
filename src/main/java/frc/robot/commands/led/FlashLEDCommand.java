package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.led.TeensyLED;

public class FlashLEDCommand extends Command {

    private TeensyLED teensyLED;
    private Color color;

    public FlashLEDCommand(TeensyLED teensyLED, Color color) {
        this.teensyLED = teensyLED;
        this.color = color;
    }

    @Override
    public void initialize() {
        super.initialize();
        teensyLED.setAll(color);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        teensyLED.setAll(Color.kBlack);
    }
}
