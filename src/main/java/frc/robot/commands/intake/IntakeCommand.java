package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends LoggedCommand {

    private IntakeSubsystem intakeSubsystem;

    private double power;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double power) {
        this.intakeSubsystem = intakeSubsystem;
        this.power = power;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeMotor(power);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        intakeSubsystem.setIntakeMotor(0);
    }
}
