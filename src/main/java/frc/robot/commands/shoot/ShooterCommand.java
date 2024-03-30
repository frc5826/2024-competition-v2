package frc.robot.commands.shoot;

import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends LoggedCommand {

    private ShooterSubsystem shooterSubsystem;
    private double speed;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double speed) {
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        super.execute();
        shooterSubsystem.setShooterOutput(speed);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooterSubsystem.setShooterOutput(0);
    }

}
