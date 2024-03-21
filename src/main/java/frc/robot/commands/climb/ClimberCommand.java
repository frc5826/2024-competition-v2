package frc.robot.commands.climb;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends LoggedCommand {

    private ClimberSubsystem climberSubsystem;

    private boolean leftEscaped, rightEscaped;
    private double power;

    public ClimberCommand(ClimberSubsystem climberSubsystem, double power) {
        this.climberSubsystem = climberSubsystem;
        this.power = power;

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        leftEscaped = false;
        rightEscaped = false;

        climberSubsystem.setLeftMotorSpeed(power);
        climberSubsystem.setRightMotorSpeed(power);
    }

    @Override
    public void execute() {
        if (!climberSubsystem.getLeftLimitSwitch()) {
            leftEscaped = true;
        }
        if (!climberSubsystem.getRightLimitSwitch()) {
            rightEscaped = true;
        }

        if (leftEscaped && climberSubsystem.getLeftLimitSwitch()) {
            climberSubsystem.setLeftMotorSpeed(0);
        }
        if (rightEscaped && climberSubsystem.getRightLimitSwitch()) {
            climberSubsystem.setRightMotorSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        climberSubsystem.setRightMotorSpeed(0);
        climberSubsystem.setLeftMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() &&
                leftEscaped && climberSubsystem.getLeftLimitSwitch() &&
                rightEscaped && climberSubsystem.getRightLimitSwitch();
    }
}
