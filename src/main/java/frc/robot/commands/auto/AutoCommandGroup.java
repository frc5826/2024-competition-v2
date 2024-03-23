package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.PathWithStopDistance;
import frc.robot.commands.drive.TurnToCommand;
import frc.robot.commands.shoot.ShootCommand;
import frc.robot.subsystems.*;

import static frc.robot.positioning.FieldOrientation.getOrientation;

public class AutoCommandGroup extends SequentialCommandGroup {

    private LocalizationSubsystem localizationSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private ArmSubsystem armSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private int ringCount;

    public AutoCommandGroup(LocalizationSubsystem localizationSubsystem, SwerveSubsystem swerveSubsystem,
                            ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
                            int ringCount, Pose2d endPose, Pose2d... rings) {

        this.armSubsystem = armSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.localizationSubsystem = localizationSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.ringCount = ringCount;

        if (getOrientation().isValid()) {

            addCommands(
                    new InstantCommand(() -> {
                        //TODO is this causing problems?
                        swerveSubsystem.setGyro(new Rotation3d(0, 0, localizationSubsystem.getCurrentPose().getRotation().getRadians() + (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Math.PI : 0 )));
                        System.err.println("Auto Started at: "+localizationSubsystem.getCurrentPose());
                    }),

                    //Shoots initial ring
                    Commands.parallel(
                            //aim speaker
                            new ShootCommand(shooterSubsystem, intakeSubsystem)
                    ));

                    //For every ring selected
                    for (int i = 0; i < ringCount; i++) {
                        Pose2d ring = rings[i];

                        if (ring != getOrientation().getNothingPose()) {

                            addCommands(
                                    new PathWithStopDistance(localizationSubsystem, ring, 1.5)
                                            .onlyIf(() -> ring.getTranslation().getDistance(localizationSubsystem.getCurrentPose().getTranslation()) > 2),

                                    new TurnToCommand(localizationSubsystem, swerveSubsystem, ring)

                                    //TODO pickup sequence including drive to ring

                                    //TODO path within speaker distance while preparing shooter for target location

                            );

                        }
                    }
            } else {
            System.err.println("ERROR: AUTO FAILED, INVALID ROBOT ORIENTATION\nRobot might be in Narnia for all I know");
            }

    }

}
