package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends LoggedSubsystem {

    CANSparkMax intakeMotor;

    DigitalInput beamBreakIntake;
    DigitalInput beamBreakShooter;

    boolean hasRing;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(intakeID, CANSparkMax.MotorType.kBrushless);
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeMotor.setInverted(false);

        beamBreakIntake = new DigitalInput(intakeBeamBreak);
        beamBreakShooter = new DigitalInput(shooterBeamBreak);
    }

    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }

    public boolean getBeamBreakIntake(){
        return beamBreakIntake.get();
    }

    public boolean getBeamBreakShooter(){
        return beamBreakShooter.get();
    }

    public void setHasRing(boolean hasRing) {
        this.hasRing = hasRing;
    }

    public boolean hasRing() {
        return hasRing;
    }

}
