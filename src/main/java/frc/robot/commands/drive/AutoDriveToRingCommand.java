package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommand;
import frc.robot.math.PID;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.XboxControllerSubsystem;
import frc.robot.vision.RingResult;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.ChronoUnit;

public class AutoDriveToRingCommand extends LoggedCommand {

    private LocalizationSubsystem localizationSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private XboxControllerSubsystem xboxControllerSubsystem;

    private boolean seesRing;

    private RingResult leftTarget;
    private RingResult rightTarget;

    private PID turnPID = new PID(Constants.cTurnPID, 3, 0.01, 0.01, this::getAverageYaw);

    private Instant initialTime;

    private double speed;

    public AutoDriveToRingCommand(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem,
                                  IntakeSubsystem intakeSubsystem, XboxControllerSubsystem xboxControllerSubsystem) {
        this.localizationSubsystem = localizationSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xboxControllerSubsystem = xboxControllerSubsystem;

        addRequirements(swerveSubsystem, xboxControllerSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        leftTarget = localizationSubsystem.getBestLeftRing();
        rightTarget = localizationSubsystem.getBestRightRing();

        turnPID.setGoal(0);

        initialTime = Instant.now();
    }

    @Override
    public void execute() {
        super.execute();

        leftTarget = localizationSubsystem.getBestLeftRing();
        rightTarget = localizationSubsystem.getBestRightRing();

//        Double rumbleMs = getRumble();
//        if (rumbleMs == null) {
//            xboxControllerSubsystem.set(0);
//        } else {
//            xboxControllerSubsystem.set(rumbleMs.intValue());
//        }

        seesRing = !leftTarget.getFieldPose().equals(new Translation2d(0, 0)) ||
                !rightTarget.getFieldPose().equals(new Translation2d(0, 0));

        //System.out.println(turnPID.getError());

        if (Math.min(leftTarget.getPitch(), rightTarget.getPitch()) < -5) {
            speed = 1;
        } else {
            speed = Constants.trackRingVel;
        }

        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(speed, 0, turnPID.calculate()));
    }

    private double getAverageYaw() {
        double leftYaw = leftTarget.getYaw();
        double rightYaw = rightTarget.getYaw();

        return Math.toRadians((leftYaw + rightYaw) / 2);
    }

//    private Double getRumble() {
//        double area = Math.max(leftTarget.getArea(), rightTarget.getArea());
//
//        return area != 0 ? Math.min(50 / area, 1000)  : null;
//    }

    @Override
    public boolean isFinished() {
        boolean noRing = !seesRing;// && (initialTime == null || Duration.between(initialTime, Instant.now()).abs().get(ChronoUnit.MILLIS) > 500);
        boolean hasRing = intakeSubsystem.hasRing();

        return noRing || hasRing;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        initialTime = null;

        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(0, 0, 0));
    }
}
