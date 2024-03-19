package frc.robot.commands.shoot;

import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends LoggedCommand {

    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setShooterSpeed(Constants.maxShooterRPM);
        double difference1 = Math.abs(shooterSubsystem.getShooterMotor1Speed() - Constants.maxShooterRPM);
        double difference2 = Math.abs(shooterSubsystem.getShooterMotor2Speed() - Constants.maxShooterRPM);

        if (difference1 < 300 && difference2 < 300) {
            intakeSubsystem.setIntakeMotor(-1);
        }
    }

    @Override
    public boolean isFinished() {
        return !intakeSubsystem.getBeamBreakShooter() && !intakeSubsystem.getBeamBreakIntake();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterSubsystem.setShooterSpeed(0);
        intakeSubsystem.setIntakeMotor(0);
    }
}
