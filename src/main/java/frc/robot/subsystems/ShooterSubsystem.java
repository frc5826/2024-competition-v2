package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.*;

import java.util.Map;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends LoggedSubsystem {

    CANSparkMax shooterMotor1, shooterMotor2;

    private boolean hasRing;

    private ShuffleboardTab tab = Shuffleboard.getTab("Manual Shot");

    private SimpleWidget lamePower = tab.add("Manual Power", 1)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 2, "publish_all", true))
            .withSize(2, 1).withPosition(0, 0);

    private SimpleWidget lameAngle = tab.add("Manual Angle", 48)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 90, "publish_all", true))
            .withSize(2, 1).withPosition(0, 1);

    public ShooterSubsystem() {
        shooterMotor1 = new CANSparkMax(shooterMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(shooterMotor2ID, CANSparkLowLevel.MotorType.kBrushless);

        shooterMotor1.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();

        shooterMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shooterMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shooterMotor1.setSmartCurrentLimit(40);
        shooterMotor2.setSmartCurrentLimit(40);

//        shooterMotor1.setOpenLoopRampRate(0);
//        shooterMotor2.setOpenLoopRampRate(0);

        shooterMotor1.setInverted(true);
        shooterMotor2.setInverted(false);

        shooterMotor1.getPIDController().setP(0.0005);
        shooterMotor2.getPIDController().setP(0.0005);

        shooterMotor1.getPIDController().setI(1e-6);
        shooterMotor2.getPIDController().setI(1e-6);

        shooterMotor1.getPIDController().setD(2e-6);
        shooterMotor2.getPIDController().setD(2e-6);

        shooterMotor1.getPIDController().setFF(0);
        shooterMotor2.getPIDController().setFF(0);

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

    public double getLamePower(){
        return lamePower.getEntry().getDouble(1);
    }

    public double getLameAngle(){
        return lameAngle.getEntry().getDouble(48);
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
