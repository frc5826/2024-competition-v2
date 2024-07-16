package frc.robot.commands.arm;

import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SuppliedArmCommand extends LoggedCommand {

    private ArmSubsystem armSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private double slop;

    public SuppliedArmCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem) {
        this(armSubsystem, shooterSubsystem, Constants.armErrorTolerance);
    }

    public SuppliedArmCommand(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, double slop) {
        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.slop = slop;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        armSubsystem.setDesiredArmAngle(shooterSubsystem.getLameAngle());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getPIDError()) < slop;
    }
}
