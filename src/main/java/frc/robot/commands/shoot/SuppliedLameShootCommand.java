package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.SuppliedArmCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SuppliedLameShootCommand extends SequentialCommandGroup {

    ArmSubsystem armSubsystem;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;

    public SuppliedLameShootCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addCommands(
                Commands.parallel(
                        new IntakeCommand(intakeSubsystem, 0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter),
                        new ShooterCommand(shooterSubsystem, -0.1).onlyWhile(intakeSubsystem::getBeamBreakShooter),
                        new SuppliedArmCommand(armSubsystem, shooterSubsystem)
                ),
                new SuppliedShootCommand(shooterSubsystem, intakeSubsystem)
        );
    }

}
