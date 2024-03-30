package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.math.ShooterMath;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnToCommand extends LoggedCommand {

    private LocalizationSubsystem localizationSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private final Pose2d pose;

    private PID turnPID = new PID(Constants.cTurnPID, 6, 1, 0.005, this::getTurn);
    private boolean speaker;

    public TurnToCommand(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem, Pose2d poseToTurnTo, boolean speaker) {
        this.localizationSubsystem = localizationSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.pose = poseToTurnTo;
        this.speaker = speaker;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0, 0, turnPID.calculate()));
    }

    @Override
    public void initialize() {
        turnPID.setGoal(0);
    }

    private double getTurn() {
        return -ShooterMath.fixSpin(pose.getTranslation()
                .minus(localizationSubsystem.getCurrentPose().getTranslation()).getAngle().getRadians()
                - localizationSubsystem.getCurrentPose().getRotation().getRadians() - (speaker ? Math.PI : 0));

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getTurn()) < 0.025;
    }
}
