package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.vision.RingResult;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoPickupRing extends LoggedCommand {

    private LocalizationSubsystem localizationSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private PID turnPID = new PID(Constants.cTurnPID, 3, 0.01, 0.01, this::ringYaw);

    private PID drivePID = new PID(Constants.cDrivePID, 1.25, 0.01, 0.01, this::ringDistance);

    private RingResult ringTracking;
    private double previousRingDistance;

    private double epsilon = .15;

    private double stopDistance = 0;

    private boolean die;

    public AutoPickupRing(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem) {
        this.localizationSubsystem = localizationSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        die = false;
        ringTracking = RingResult.getEmpty(); // localizationSubsystem.getBestPickupRing();
        if (ringTracking.getFieldPose().equals(new Translation2d(0, 0))) {
            die = true;
        }

        turnPID.setGoal(0);
        drivePID.setGoal(0);
    }

    @Override
    public void execute() {
        RingResult bestRing = RingResult.getEmpty(); // localizationSubsystem.getBestPickupRing();
        

        if (bestRing.getDistance() - previousRingDistance < epsilon) {
            ringTracking = bestRing;
        }

        previousRingDistance = ringTracking.getDistance();

        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(-drivePID.calculate(), 0, turnPID.calculate()));
    }

    private double ringYaw() {
        //System.out.println(ringTracking.getFieldPose().minus(localizationSubsystem.getCurrentPose().getTranslation()).rotateBy(localizationSubsystem.getCurrentPose().getRotation().unaryMinus()).getAngle().getRadians());
        //return ringTracking.getFieldPose().minus(localizationSubsystem.getCurrentPose().getTranslation()).rotateBy(localizationSubsystem.getCurrentPose().getRotation().times(-1)).getAngle().getRadians() * -1;
        return ringTracking.getYaw();
    }

    private double ringDistance() {
        return ringTracking.getDistance();
        //return ringTracking.getFieldPose().getDistance(localizationSubsystem.getCurrentPose().getTranslation()) - stopDistance;
    }

    @Override
    public boolean isFinished() {
        return ringDistance() < 0.1 || die;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSubsystem.driveFieldOriented(new ChassisSpeeds(0,0,0));
    }
}
