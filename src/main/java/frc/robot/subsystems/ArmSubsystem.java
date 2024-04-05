package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.ArmController;
import frc.robot.math.PID;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends LoggedSubsystem {

    private CANSparkMax rotateMotor, rotateMotorSecondary;

    private DutyCycleEncoder rotateEncoder;

    private PID rotatePID;
    private ArmController controller;
    private double desiredArmRotations;

    private SimpleWidget speed;

    public ArmSubsystem() {
        rotateMotor = new CANSparkMax(rotateMotor1ID, CANSparkLowLevel.MotorType.kBrushless);
        rotateMotorSecondary = new CANSparkMax(rotateMotor2ID, CANSparkLowLevel.MotorType.kBrushless);

        rotateMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rotateMotorSecondary.setIdleMode(CANSparkBase.IdleMode.kBrake);

        rotateMotor.setInverted(false);
        rotateMotorSecondary.setInverted(false);

        rotateMotor.setSmartCurrentLimit(40);
        rotateMotorSecondary.setSmartCurrentLimit(40);

        rotateMotorSecondary.follow(rotateMotor);

        rotateEncoder = new DutyCycleEncoder(rotateEncoderID);

        rotatePID = new PID(cRotateP, cRotateI, cRotateD, cRotateMax, cRotateMin, cRotateDeadband, this::getRotation);

        //TODO
        //controller = new ArmController(cRotateP, cRotateI, cRotateMax, cGravityConstant, cVelConstant, cMaxVel, cMaxAccel, this::getRotation);

        setArmHome();
        setupArmTab();
    }

    private void setupArmTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("ARM");

        tab.addNumber("Rotation", this::getRotation);
        tab.addNumber("Absolute Rotation", this::getRotationAbsolute);
        tab.addNumber("Arm Desired Rotation", () -> desiredArmRotations);

        speed = tab.add("speed", 0);

        tab.add("pid", rotatePID);
        //tab.add("armController", controller);


    }

    @Override
    public void periodic() {
        rotatePID.setGoal(desiredArmRotations);

        double rotate = rotatePID.calculate();

        setRotateSpeed(clamp(rotate + Math.cos(desiredArmRotations * Math.PI * 2) * cGravityConstant, -1, 1));

        //setRotateSpeed(controller.calculate());//TODO
    }

    private double clamp(double input, double bound1, double bound2){
        if(bound2 >= bound1) return Math.max(Math.min(input, bound2), bound1);
        return Math.max(Math.min(input, bound1), bound2);
    }

    public void setDesiredArmAngle(double armAngleDegrees){
        double clampedAngle = clamp(armAngleDegrees, -8, 110);
        desiredArmRotations = clampedAngle / 360;
        rotatePID.setGoal(desiredArmRotations);
//        controller.setGoal(desiredArmRotations);
        rotatePID.calculate();
    }

    public void setArmHome(){
        desiredArmRotations = 0;
    }

    public void setRotateSpeed(double speed){
        rotateMotor.set(speed);
    }

    public double getRotation(){
        return -rotateEncoder.getAbsolutePosition() + armOffset;
    }

    private double getDifference(double rotation) {
        rotation -= 0.5;

        if (rotation < 0.5) {
            rotation++;
        } else if (rotation > 0.5) {
            rotation--;
        }

        return rotation;
    }

    public double getPIDError() {
        return rotatePID.getError();
    }

    public double getRotationDegrees(){
        return getRotation() * 360;
    }

    public double getRotationAbsolute(){
        return rotateEncoder.getAbsolutePosition();
    }

    public double getDesiredArmRotations() {
        return desiredArmRotations;
    }
}
