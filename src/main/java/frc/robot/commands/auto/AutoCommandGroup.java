package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.conglomerate.ScoreCommandGroup;
import frc.robot.commands.drive.AutoDriveToRingCommand;
import frc.robot.commands.drive.PathCommand;
import frc.robot.commands.drive.PathWithStopDistance;
import frc.robot.commands.drive.TurnToCommand;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.shoot.ShootCommand;
import frc.robot.commands.shoot.ShootSpeakerCommandGroup;
import frc.robot.math.ShooterMath;
import frc.robot.positioning.FieldOrientation;
import frc.robot.positioning.Orientation;
import frc.robot.subsystems.*;

import static frc.robot.Constants.*;
import static frc.robot.positioning.FieldOrientation.getOrientation;

public class AutoCommandGroup extends SequentialCommandGroup {

    private LocalizationSubsystem localizationSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private ArmSubsystem armSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private XboxControllerSubsystem xboxControllerSubsystem;

    private int ringCount;

    public AutoCommandGroup(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem,
                            ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem,
                            IntakeSubsystem intakeSubsystem, XboxControllerSubsystem xboxControllerSubsystem,
                            Pose2d endPose, Pose2d shootPose, int ringCount, Pose2d... rings) {

        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.localizationSubsystem = localizationSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.xboxControllerSubsystem = xboxControllerSubsystem;

        this.ringCount = ringCount;

        //addCommands(new ArmCommand(armSubsystem, homeArmAngle));

        if (getOrientation().isValid()) {

            addCommands(
                    new InstantCommand(() -> {
                        //TODO is this causing problems?
                        swerveSubsystem.setGyro(new Rotation3d(0, 0, localizationSubsystem.getCurrentPose().getRotation().getRadians()));
                        localizationSubsystem.reset(localizationSubsystem.getCurrentPose());
                    }),

                    //Shoots initial ring
                    new ShootSpeakerCommandGroup(shooterSubsystem, intakeSubsystem
                            , armSubsystem, localizationSubsystem, swerveSubsystem));

                    //For every ring selected
                    for (int i = 0; i < ringCount; i++) {
                        Pose2d ring = rings[i];

                        if (ring != getOrientation().getNothingPose()) {

                            addCommands(

                                    Commands.parallel(
                                            new ArmCommand(armSubsystem, -5, .04),
                                            new TurnToCommand(localizationSubsystem, swerveSubsystem, ring, false)
                                    ),
                                    new PathWithStopDistance(localizationSubsystem, ring, 2).onlyIf(() -> ring.getTranslation().getDistance(localizationSubsystem.getCurrentPose().getTranslation()) > 2.5),

                                    new WaitCommand(.5).until(localizationSubsystem::seesRing),

                                    Commands.parallel(
                                    new AutoDriveToRingCommand(localizationSubsystem, swerveSubsystem, intakeSubsystem, xboxControllerSubsystem),
                                    new IntakeCommandGroup(intakeSubsystem, armSubsystem, shooterSubsystem).until(() -> (localizationSubsystem.timeSinceSeenRing() > 2))
                                    ).onlyIf(localizationSubsystem::seesRing),

//                                    new PathWithStopDistance(localizationSubsystem, FieldOrientation.getOrientation().getSpeakerTargetPos(),
//                                            3, true).onlyIf(intakeSubsystem::hasRing),

//                                    new PathCommand(FieldOrientation::getOrientation, orientation -> {
//                                        Translation2d shootTranslation = shootPose.getTranslation();
//                                        Rotation2d speakerRotation = Rotation2d.fromRadians(ShooterMath.getSpeakerTurn(orientation.getSpeakerTargetPos(), shootPose, true));
//
//                                        System.out.println("AUTO PATH - TARGET POSE: " + orientation.getSpeakerTargetPos());
//                                        System.out.println("AUTO PATH - CURRENT POSE: " + shootPose);
//                                        System.out.println("AUTO PATH - SPEAKER: " + true);
//
//                                        return new Pose2d(shootTranslation, speakerRotation);
//                                    }, localizationSubsystem).onlyIf(intakeSubsystem::hasRing),
//                                    new ShootSpeakerCommandGroup(shooterSubsystem, intakeSubsystem, armSubsystem,
//                                            localizationSubsystem, swerveSubsystem).onlyIf(intakeSubsystem::hasRing)
//
//                            );
                                    new ScoreCommandGroup(armSubsystem, intakeSubsystem, shooterSubsystem, localizationSubsystem, swerveSubsystem).onlyIf(intakeSubsystem::hasRing));

                        }
                    }

                    addCommands(
                            new ArmCommand(armSubsystem, -5, .02),
                            new PathCommand(FieldOrientation::getOrientation, orientation -> endPose, localizationSubsystem)
                    );
            } else {
            System.err.println("ERROR: AUTO FAILED, INVALID ROBOT ORIENTATION\nRobot might be in Narnia for all I know");
            }

    }

}
