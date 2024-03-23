package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeakerCommandGroup extends SequentialCommandGroup {

    public ShootSpeakerCommandGroup(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                Commands.parallel(
                        new IntakeCommand(intakeSubsystem, 0.1),
                        new ShooterCommand(shooterSubsystem, -0.1)
                ).onlyWhile(intakeSubsystem::getBeamBreakShooter),
                new ShootCommand(shooterSubsystem, intakeSubsystem),
                new RunCommand(() -> intakeSubsystem.setHasRing(false))
        );
    }
}
