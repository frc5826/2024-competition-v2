package frc.robot.commands.conglomerate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.TargetSpeakerCommand;
import frc.robot.commands.drive.PathThenFollowCommand;
import frc.robot.commands.drive.TurnToCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shoot.ShootCommand;
import frc.robot.commands.shoot.ShooterCommand;
import frc.robot.commands.shoot.SpinupShooterCommand;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.subsystems.*;

import java.util.function.Function;

import static frc.robot.Constants.shootArmAngle;

public class ScoreCommandGroup extends SequentialCommandGroup {

    public ScoreCommandGroup(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
                             LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem,
                             Function<Orientation, Pose2d> firstPathPose, Function<Orientation, Pose2d> finalPathPose) {

        addCommands(
                new PathThenFollowCommand(firstPathPose, finalPathPose, FieldOrientation::getOrientation, localizationSubsystem)
                        .alongWith(new ArmCommand(armSubsystem, shootArmAngle))
                        .deadlineWith(Commands.parallel(
                                new IntakeCommand(intakeSubsystem, 0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter),
                                new ShooterCommand(shooterSubsystem, -0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter)
                        ).andThen(new SpinupShooterCommand(shooterSubsystem)))
                        .andThen(Commands.parallel(new TargetSpeakerCommand(armSubsystem, localizationSubsystem),
                                new TurnToCommand(localizationSubsystem, swerveSubsystem, () -> FieldOrientation.getOrientation().getSpeakerTargetPos(), true)))
                        .andThen(new ShootCommand(shooterSubsystem, intakeSubsystem))
        );

    }
}
