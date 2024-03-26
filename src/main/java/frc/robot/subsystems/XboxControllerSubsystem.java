package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;

public class XboxControllerSubsystem extends SubsystemBase {

    private int pulseMs = 0;
    private int pauseMs = 0;

    private Instant lastPulse = Instant.MIN;
    private Instant lastPause = Instant.MIN;

    public XboxControllerSubsystem() {

    }

    @Override
    public void periodic() {
        super.periodic();

        if (Duration.between(lastPulse, Instant.now()).abs().get(ChronoUnit.MILLIS) >= pauseMs) {
            Constants.xbox.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            lastPause = Instant.now();
        } else if (Duration.between(lastPause, Instant.now()).abs().get(ChronoUnit.MILLIS) >= pulseMs) {
            Constants.xbox.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
            lastPulse = Instant.now();
        }

    }

    public int getPulseMs() {
        return pulseMs;
    }

    public void setPulseMs(int pulseMs) {
        this.pulseMs = pulseMs;
    }

    public int getPauseMs() {
        return pauseMs;
    }

    public void setPauseMs(int pauseMs) {
        this.pauseMs = pauseMs;
    }

    public void set(int ms) {
        pauseMs = ms;
        pulseMs = ms;
    }
}
