package frc.robot.commands.shoot;

import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinupShooterCommand extends LoggedCommand {

    private ShooterSubsystem shooterSubsystem;

    public SpinupShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        super.execute();

        shooterSubsystem.setShooterSpeed(Constants.maxShooterRPM);
    }

    @Override
    public boolean isFinished() {
        double difference1 = Math.abs(shooterSubsystem.getShooterMotor1Speed() - Constants.maxShooterRPM);
        double difference2 = Math.abs(shooterSubsystem.getShooterMotor2Speed() - Constants.maxShooterRPM);

        boolean toleranceMet = difference1 < Constants.shooterTolerance && difference2 < Constants.shooterTolerance;

        return toleranceMet;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        if (interrupted) {
            shooterSubsystem.setShooterOutput(Constants.maxShooterRPM);
        }
    }
}
