package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandGroup extends SequentialCommandGroup {

    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;

    public IntakeCommandGroup(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;

        addCommands(

                new IntakeCommand(intakeSubsystem, -1).until(intakeSubsystem::getBeamBreakIntake),
                Commands.parallel(
                        new ArmCommand(armSubsystem, 10),
                        new IntakeCommand(intakeSubsystem, -1).until(intakeSubsystem::getBeamBreakShooter)
                ),
                new IntakeCommand(intakeSubsystem, 0.2).onlyWhile(intakeSubsystem::getBeamBreakShooter)

        );
    }

}
