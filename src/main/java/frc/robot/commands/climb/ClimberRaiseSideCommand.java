package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRaiseSideCommand extends LoggedCommand {
    private ClimberSubsystem climberSubsystem;
    private ClimberSide climberSide;

    public ClimberRaiseSideCommand(ClimberSubsystem climberSubsystem, ClimberSide climberSide) {
        this.climberSubsystem = climberSubsystem;
        this.climberSide = climberSide;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        switch (climberSide){
            case LEFT -> climberSubsystem.setLeftMotorSpeed(0.2);
            case RIGHT -> climberSubsystem.setRightMotorSpeed(0.2);
        }
    }

    @Override  //Does this call when limit switch is "switched"?
    public boolean isFinished() {
        switch (climberSide){
            case LEFT -> {
                return !climberSubsystem.getLeftLimitSwitch();
            }
            case RIGHT -> {
                return !climberSubsystem.getRightLimitSwitch();
            }
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        switch (climberSide){
            case LEFT -> climberSubsystem.setLeftMotorSpeed(0);
            case RIGHT -> climberSubsystem.setRightMotorSpeed(0);
        }
    }

    public enum ClimberSide{
        LEFT,
        RIGHT,
    }
}




