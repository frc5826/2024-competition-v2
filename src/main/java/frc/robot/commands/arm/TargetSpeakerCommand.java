package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.ShooterMath;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;

public class TargetSpeakerCommand extends LoggedCommand {

    private ArmSubsystem armSubsystem;
    private LocalizationSubsystem localizationSubsystem;

    public TargetSpeakerCommand(ArmSubsystem armSubsystem, LocalizationSubsystem localizationSubsystem) {
        this.armSubsystem = armSubsystem;
        this.localizationSubsystem = localizationSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        armSubsystem.setDesiredArmAngle(ShooterMath.getShootingAngle(
                localizationSubsystem.getCurrentPose(), FieldOrientation.getOrientation().getSpeakerTargetPos()));
    }

    @Override
    public boolean isFinished() {
        boolean armFinished = Math.abs(armSubsystem.getPIDError()) < Constants.armErrorTolerance;
        return armFinished;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
