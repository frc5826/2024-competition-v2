package frc.robot.commands.drive;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.LoggedCommand;
import frc.robot.subsystems.LocalizationSubsystem;

public class PathWithStopDistance extends LoggedCommand {
    private LocalizationSubsystem localizationSubsystem;
    private Pose2d ringPose;
    private double stopDistance;

    private boolean inverted = false;

    private Command buildCommand;
    public PathWithStopDistance(LocalizationSubsystem localizationSubsystem, Pose2d ringPose, double stopDistance) {
        this(localizationSubsystem, ringPose, stopDistance, false);
    }

    public PathWithStopDistance(LocalizationSubsystem localizationSubsystem, Pose2d ringPose,
                                double stopDistance, boolean inverted) {
        this.localizationSubsystem = localizationSubsystem;

        this.ringPose = ringPose;

        this.stopDistance = stopDistance;

        this.inverted = inverted;
    }

    @Override
    public void initialize() {
        super.initialize();
        buildCommand = localizationSubsystem.buildPath(ringPose);
        //CommandScheduler.getInstance().schedule(buildCommand);
        buildCommand.initialize();
    }

    @Override
    public void execute() {
        buildCommand.execute();

        double offset = inverted ? 180 : 0;

        localizationSubsystem.setRotationTarget(ringPose.getTranslation()
                .minus(localizationSubsystem.getCurrentPose().getTranslation()).getAngle().minus(Rotation2d.fromDegrees(offset)));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        localizationSubsystem.removeRotationTarget();
        //CommandScheduler.getInstance().cancel(buildCommand);
        buildCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
            Pose2d currentPose = localizationSubsystem.getCurrentPose();
            return currentPose.getTranslation().getDistance(ringPose.getTranslation()) <= stopDistance;
    }
}
