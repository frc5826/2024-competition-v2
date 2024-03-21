package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.shoot.ShootCommand;
import frc.robot.commands.shoot.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommandGroup extends SequentialCommandGroup {

    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public IntakeCommandGroup(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addCommands(

                Commands.parallel(
                        new ArmCommand(armSubsystem, Constants.intakeArmAngle),
                        new IntakeCommand(intakeSubsystem, -1).until(intakeSubsystem::getBeamBreakIntake)
                ),
                Commands.parallel(
                        new ArmCommand(armSubsystem, 10),
                        new IntakeCommand(intakeSubsystem, -0.5).until(intakeSubsystem::getBeamBreakShooter)
                ),
                Commands.parallel(
                        new IntakeCommand(intakeSubsystem, 0.1),
                        new ShooterCommand(shooterSubsystem, -0.1)
                ).onlyWhile(intakeSubsystem::getBeamBreakShooter)
        );
    }

}
