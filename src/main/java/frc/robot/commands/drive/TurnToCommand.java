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

import java.util.function.Supplier;

public class TurnToCommand extends LoggedCommand {

    private LocalizationSubsystem localizationSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private final Supplier<Pose2d> pose;

    private PID turnPID = new PID(Constants.cTurnPID, 6, 1.25, 0.005, this::getTurn);
    private boolean speaker;

    public TurnToCommand(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem, Supplier<Pose2d> poseToTurnTo, boolean speaker) {
        this.localizationSubsystem = localizationSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.pose = poseToTurnTo;
        this.speaker = speaker;
        addRequirements(swerveSubsystem);
    }

    public TurnToCommand(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem, Pose2d poseToTurnTo, boolean speaker) {
        this(localizationSubsystem,swerveSubsystem,() -> poseToTurnTo,speaker);
    }

    @Override
    public void execute() {
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0, 0, turnPID.calculate()));
    }

    @Override
    public void initialize() {
        super.initialize();
        turnPID.setGoal(0);
        if(pose.get() == null) {
            System.err.println("Turn to speaker command was given an invalid location!!!!");
            this.cancel();
        }
    }

    private double getTurn() {
//        return -ShooterMath.fixSpin(pose.get().getTranslation()
//                .minus(localizationSubsystem.getCurrentPose().getTranslation()).getAngle().getRadians()
//                - localizationSubsystem.getCurrentPose().getRotation().getRadians() - (speaker ? Math.PI : 0));

        Pose2d current = localizationSubsystem.getCurrentPose();
        double turnto = ShooterMath.getSpeakerTurn(pose.get(), current, speaker);
        return turnto;

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getTurn()) < 0.007;
    }
}
