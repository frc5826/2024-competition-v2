package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends LoggedSubsystem {

    CANSparkMax shooterMotor1, shooterMotor2;

    private boolean hasRing;

    public ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(shooterMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(shooterMotor2ID, CANSparkLowLevel.MotorType.kBrushless);

        shooterMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooterMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(false);

        shooterMotor1.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();

        shooterMotor1.getPIDController().setP(24e-5);
        shooterMotor2.getPIDController().setP(24e-5);

        shooterMotor1.getPIDController().setI(1e-6);
        shooterMotor2.getPIDController().setI(1e-6);

        shooterMotor1.getPIDController().setD(4e-6);
        shooterMotor2.getPIDController().setD(4e-6);

        shooterMotor1.getPIDController().setFF(30e-6);
        shooterMotor2.getPIDController().setFF(30e-6);

        shooterMotor1.getPIDController().setIZone(0);
        shooterMotor2.getPIDController().setIZone(0);

        shooterMotor1.getPIDController().setOutputRange(-1, 1);
        shooterMotor2.getPIDController().setOutputRange(-1, 1);

        setupShooterTab();
    }

    private void setupShooterTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("SHOOTER");

        tab.addNumber("1 Speed", this::getShooterMotor1Speed);
        tab.addNumber("2 Speed", this::getShooterMotor2Speed);
    }

    @Override
    public void periodic() {

    }

    public void setShooterOutput(double speed){
        setShooterOutput(speed, true, true);
    }

    public void setShooterOutput(double speed, boolean top, boolean bottom){
        if(top) {
            shooterMotor1.set(speed);
        }
        if(bottom) {
            shooterMotor2.set(speed);
        }
    }

    public void setShooterSpeed(double speed){
        shooterMotor1.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
        shooterMotor2.getPIDController().setReference(speed, CANSparkBase.ControlType.kVelocity);
    }

    public CANSparkMax getShooterMotor1() {
        return shooterMotor1;
    }

    public CANSparkMax getShooterMotor2() {
        return shooterMotor2;
    }

    public double getShooterMotor1Speed(){
        return shooterMotor1.getEncoder().getVelocity();
    }

    public double getShooterMotor2Speed(){
        return shooterMotor2.getEncoder().getVelocity();
    }
}
