package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LameShootCommand extends SequentialCommandGroup {

    ArmSubsystem armSubsystem;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;

    public LameShootCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addCommands(
                Commands.parallel(
                        new IntakeCommand(intakeSubsystem, 0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter),
                        new ShooterCommand(shooterSubsystem, -0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter),
                        new ArmCommand(armSubsystem, Constants.shootArmAngle)
                ),
                new ShootCommand(shooterSubsystem, intakeSubsystem)
        );
    }

}
