package frc.robot.math;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.function.DoubleSupplier;

public class ArmController implements Sendable {
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpoint;
    private PID pid;
    private double G,V,goal;
    private final DoubleSupplier actual;

    public ArmController(double P, double I, double D, double G, double V, double maxVel, double maxAccel, DoubleSupplier actual) {
        pid = new PID(P,I,D,1,-1,0, actual);
        this.G = G;
        this.V = V;
        this.actual = actual;
        constraints = new TrapezoidProfile.Constraints(maxVel,maxAccel);
        profile = new TrapezoidProfile(constraints);
        setpoint = new TrapezoidProfile.State(actual.getAsDouble(),0);
    }

    public double calculate() {
        setpoint = profile.calculate(0.2,
                setpoint,
                new TrapezoidProfile.State(goal,0)
        );
        pid.setGoal(setpoint.position);
        return pid.calculate() + calculateFeedForward(actual.getAsDouble(), setpoint.velocity);
    }

    public void setGoal(double goal) {
        this.goal = goal;
        setpoint = new TrapezoidProfile.State(actual.getAsDouble(), 0);
    }

    public void setG(double g) {
        G = g;
    }

    public double getG() {
        return G;
    }

    public void setV(double v) {
        V = v;
    }

    public double getV() {
        return V;
    }

    public void setMaxVel(double vel) {
        constraints = new TrapezoidProfile.Constraints(vel, constraints.maxAcceleration);
        profile = new TrapezoidProfile(constraints);
    }

    public double getMaxVel() {
        return constraints.maxVelocity;
    }

    public double getVelocitySetpoint() {
        return setpoint.velocity;
    }

    public double getPositionSetpoint() {
        return setpoint.position;
    }

    public void setMaxAccel(double accel) {
        constraints = new TrapezoidProfile.Constraints(constraints.maxVelocity, accel);
        profile = new TrapezoidProfile(constraints);
    }

    public double getMaxAccel() {
        return constraints.maxAcceleration;
    }

    public double getGoal() {
        return goal;
    }

    private double calculateFeedForward(double position, double vel) {
        return Math.cos(position*2*Math.PI) * G + vel * V;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        pid.initSendable(builder);
        builder.addDoubleProperty("G", this::getG, this::setG);
        builder.addDoubleProperty("V", this::getV, this::setV);
        builder.addDoubleProperty("VelSetpoint", this::getVelocitySetpoint, null);
        builder.addDoubleProperty("PositionSetpoint", this::getPositionSetpoint,null);
        builder.addDoubleProperty("maxVel", this::getMaxVel, this::setMaxVel);
        builder.addDoubleProperty("maxAccel", this::getMaxAccel, this::setMaxAccel);
        builder.addDoubleProperty("ControllerGoal", this::getGoal, this::setGoal);
    }
}
