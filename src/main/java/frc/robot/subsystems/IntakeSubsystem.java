package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class IntakeSubsystem extends LoggedSubsystem {

    CANSparkMax intakeMotor;

    DigitalInput beamBreak;

    boolean hasRing;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(intakeID, CANSparkMax.MotorType.kBrushless);
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeMotor.setInverted(false);

        beamBreak = new DigitalInput(beamBreakID);
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public void setHasRing(boolean hasRing) {
        this.hasRing = hasRing;
    }

    public boolean hasRing() {
        return hasRing;
    }

}
