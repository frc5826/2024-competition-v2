package frc.robot.commands.shoot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.TargetSpeakerCommand;
import frc.robot.commands.drive.TurnToCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.subsystems.*;

public class ShootSpeakerCommandGroup extends SequentialCommandGroup {

    public ShootSpeakerCommandGroup(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
                                    ArmSubsystem armSubsystem, LocalizationSubsystem localizationSubsystem,
                                    SwerveSubsystem swerveSubsystem) {
        addCommands(
                Commands.parallel(
                        new IntakeCommand(intakeSubsystem, 0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter),
                        new ShooterCommand(shooterSubsystem, -0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter)
                ),
                Commands.deadline(
                        Commands.parallel(
                                new TargetSpeakerCommand(armSubsystem, localizationSubsystem),
                                new TurnToCommand(localizationSubsystem, swerveSubsystem, () -> FieldOrientation.getOrientation().getSpeakerTargetPos(), true)
                        ),
                        new SpinupShooterCommand(shooterSubsystem)
                ),
                new ShootCommand(shooterSubsystem, intakeSubsystem)
        );
    }
}
