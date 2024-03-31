package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends LoggedCommand {

    private ArmSubsystem armSubsystem;
    private double goal;
    private double slop;

    public ArmCommand(ArmSubsystem armSubsystem, double goalRotations) {
        this(armSubsystem, goalRotations, Constants.armErrorTolerance);
    }

    public ArmCommand(ArmSubsystem armSubsystem, double goalRotations, double slop) {
        this.armSubsystem = armSubsystem;
        this.goal = goalRotations;
        this.slop = slop;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        armSubsystem.setDesiredArmAngle(goal);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        System.out.println("finished");
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getPIDError()) < slop;
    }
}
