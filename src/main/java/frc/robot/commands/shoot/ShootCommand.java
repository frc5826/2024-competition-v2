package frc.robot.commands.shoot;

import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;
import java.time.temporal.TemporalUnit;

public class ShootCommand extends LoggedCommand {

    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Instant initInstant;

    public ShootCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        initInstant = Instant.now();
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setShooterSpeed(Constants.maxShooterRPM);
        double difference1 = Math.abs(shooterSubsystem.getShooterMotor1Speed() - Constants.maxShooterRPM);
        double difference2 = Math.abs(shooterSubsystem.getShooterMotor2Speed() - Constants.maxShooterRPM);

        boolean toleranceMet = difference1 < Constants.shooterTolerance && difference2 < Constants.shooterTolerance;
        //If something happens and we can't spin the motors up, we still want to try shooting after some point.
        boolean timeElapsed = initInstant != null && Duration.between(initInstant, Instant.now()).abs().getSeconds() > 3;

        if(timeElapsed){
            System.out.println("WARNING - Time Elapsed on Shooters");
        }

        if (toleranceMet || timeElapsed) {
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
        shooterSubsystem.setShooterOutput(0);
        intakeSubsystem.setIntakeMotor(0);
        initInstant = null;
    }
}
