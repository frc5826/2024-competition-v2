package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LoggedCommand;
import frc.robot.commands.shoot.ShooterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeSecondHalfCommandGroup extends SequentialCommandGroup {

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public IntakeSecondHalfCommandGroup(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addCommands(
                new IntakeCommand(intakeSubsystem, -1).until(intakeSubsystem::getBeamBreakShooter),
                Commands.parallel(
                        new IntakeCommand(intakeSubsystem, 0.1),
                        new ShooterCommand(shooterSubsystem, -0.1)
                ).onlyWhile(intakeSubsystem::getBeamBreakShooter)
        );
    }


}
