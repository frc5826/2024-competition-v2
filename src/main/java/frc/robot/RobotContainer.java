// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.TargetSpeakerCommand;
import frc.robot.commands.auto.AutoCommandGroup;
import frc.robot.commands.climb.ClimberCommand;
import frc.robot.commands.drive.AutoDriveToRingCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.intake.IntakeSecondHalfCommandGroup;
import frc.robot.commands.led.FlashLEDCommand;
import frc.robot.commands.shoot.LameShootCommand;
import frc.robot.commands.shoot.ShootSpeakerCommandGroup;
import frc.robot.commands.shoot.ShooterCommand;
import frc.robot.led.TeensyLED;
import frc.robot.positioning.FieldOrientation;
import frc.robot.subsystems.*;

import java.io.File;
import java.util.ArrayList;
import java.util.Map;
import java.util.function.Supplier;

import static frc.robot.Constants.*;
import static frc.robot.positioning.FieldOrientation.getOrientation;


public class RobotContainer
{

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory() + "/swerve"));

    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final XboxControllerSubsystem xboxControllerSubsystem = new XboxControllerSubsystem();

    private final TeensyLED teensyLED = new TeensyLED(0, 8, Color.kBlack);

    private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(visionSubsystem, swerveSubsystem);

    private final TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(
            swerveSubsystem,
            ()-> -xbox.getLeftY(), ()-> -xbox.getLeftX(), ()-> -xbox.getRightX(),
            ()-> xbox.getLeftTriggerAxis(), ()-> xbox.getRightTriggerAxis());

    private ArrayList<SendableChooser<Pose2d>> autoOptions;
    private int autoRings = 0;

    private Supplier<Double> endX;
    private Supplier<Double> endY;
    private Supplier<Double> endRotation;
    private Supplier<Double> shootX;
    private Supplier<Double> shootY;
    private final Field2d field = new Field2d();
    private boolean autoInitialized = false;

    public RobotContainer()
    {
        setupButtonBoardBindings();
        setupXboxBindings();
        SignalLogger.enableAutoLogging(false);

        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, teleopDriveCommand);

        DataLogManager.start();

        setupEndPose();
        configureAutoTab();
    }

    private void setupXboxBindings() {

        //new Trigger(xbox::getBButton).whileTrue(localizationSubsystem.buildPath(FieldOrientation.getOrientation().getAmpPark()));

        new Trigger(xbox::getAButton).whileTrue(new AutoDriveToRingCommand(localizationSubsystem, swerveSubsystem, intakeSubsystem, xboxControllerSubsystem));

        new Trigger(xbox::getBackButton).and(xbox::getStartButton).debounce(1)
                .whileTrue(new RunCommand(() -> {
                    swerveSubsystem.zeroGyro();
                    xbox.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
                }).finallyDo(() -> xbox.setRumble(GenericHID.RumbleType.kBothRumble, 0)));
    }

    private void setupButtonBoardBindings() {
        panelButtons[0].onTrue(new ClimberCommand(climberSubsystem, climbDownPower));
        panelButtons[1].onTrue(new ArmCommand(armSubsystem, climbArmAngle));
        panelButtons[2].onTrue(new ClimberCommand(climberSubsystem, climbUpPower));
        panelButtons[3].whileTrue(new FlashLEDCommand(teensyLED, Color.kYellow));
        panelButtons[4].whileTrue(new IntakeSecondHalfCommandGroup(intakeSubsystem, shooterSubsystem));
        panelButtons[5].onTrue(new IntakeCommandGroup(intakeSubsystem, armSubsystem, shooterSubsystem));
        panelButtons[6].onTrue(new LameShootCommand(armSubsystem, shooterSubsystem, intakeSubsystem));
        panelButtons[7].onTrue(new ArmCommand(armSubsystem, shootArmAngle));
        panelButtons[8].whileTrue(Commands.parallel(new IntakeCommand(intakeSubsystem, 0.3), new ShooterCommand(shooterSubsystem,-0.1)));
        panelButtons[9].onTrue(new ArmCommand(armSubsystem, ampArmAngle));
        panelButtons[10].onTrue(new ArmCommand(armSubsystem, homeArmAngle));
        panelButtons[11].onTrue(new ShootSpeakerCommandGroup(shooterSubsystem, intakeSubsystem, armSubsystem, localizationSubsystem, swerveSubsystem));
        //panelButtons[11].onTrue(new RunCommand(() -> shooterSubsystem.setShooterSpeed(500)));
    }

    public void updateField() {
        field.setRobotPose(new Pose2d(endX.get(), endY.get(), Rotation2d.fromDegrees(endRotation.get())));
        field.getObject("shoot").setPoses(new Pose2d(shootX.get(), shootY.get(), Rotation2d.fromDegrees(0)));
    }

    private void setupEndPose() {

        ShuffleboardTab autoTab = Shuffleboard.getTab("auto");

        var shootingX = autoTab.add("Shoot X", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(2, 1).withPosition(3, 4)
                .withProperties(Map.of("publish_all", true));

        var shootingY = autoTab.add("Shoot Y", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(2, 1).withPosition(5, 4)
                .withProperties(Map.of("publish_all", true));

        var x = autoTab.add("End X", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(2, 1).withPosition(3, 2)
                .withProperties(Map.of("publish_all", true));

        var y = autoTab.add("End Y", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(2, 1).withPosition(3, 3)
                .withProperties(Map.of("publish_all", true));

        var rot = autoTab.add("End Rotation", 0)
                .withWidget(BuiltInWidgets.kTextView)
                .withSize(2, 1).withPosition(5, 2)
                .withProperties(Map.of("publish_all", true));

        endX = () -> x.getEntry().getDouble(0);
        endY = () -> y.getEntry().getDouble(0);
        endRotation = () -> rot.getEntry().getDouble(0);

        shootX = () -> shootingX.getEntry().getDouble(0);
        shootY = () -> shootingY.getEntry().getDouble(0);

        autoTab.add(field).withSize(3, 2).withPosition(0, 2);
    }

    public void configureAutoTab() {
        if (!autoInitialized && getOrientation().isValid()) {

            ShuffleboardTab autoTab = Shuffleboard.getTab("auto");

            autoOptions = new ArrayList<SendableChooser<Pose2d>>();

            autoRings = 0;

            autoInitialized = true;

            int widgetX = 0;
            int widgetY = 0;

            for (int i = 0; i < 8; i++) {
                autoOptions.add(i, new SendableChooser<Pose2d>());

                autoOptions.get(i).setDefaultOption("Nothing", getOrientation().getNothingPose());
                autoOptions.get(i).addOption("Top Close Ring 1", getOrientation().getTopCloseRing());
                autoOptions.get(i).addOption("Mid Close Ring 2", getOrientation().getMidCloseRing());
                autoOptions.get(i).addOption("Bot Close Ring 3", getOrientation().getBotCloseRing());
                autoOptions.get(i).addOption("Top Far Ring 4", getOrientation().getFarRing4());
                autoOptions.get(i).addOption("Mid Top Far Ring 5", getOrientation().getFarRing5());
                autoOptions.get(i).addOption("Mid Far Ring 6", getOrientation().getFarRing6());
                autoOptions.get(i).addOption("Mid Bot Far Ring 7", getOrientation().getFarRing7());
                autoOptions.get(i).addOption("Bot Far Ring 8", getOrientation().getFarRing8());

                autoTab.add("Auto " + i, autoOptions.get(i)).withWidget(BuiltInWidgets.kComboBoxChooser)
                        .withSize(2, 1).withPosition(widgetX, widgetY);

                widgetX += 2;
                if (widgetX == 8) {
                    widgetY += 1;
                    widgetX = 0;
                }
    }

    }
    }

    public Command getAutonomousCommand()
    {
        boolean endPose = false;

        Pose2d endLoc = new Pose2d(endX.get(), endY.get(), Rotation2d.fromDegrees(endRotation.get()));
        Pose2d shootPose = new Pose2d(shootX.get(), shootY.get(), Rotation2d.fromDegrees(0));

        for (int i = 0; i < 8; i++) {
            if (autoOptions.get(i).getSelected() != getOrientation().getNothingPose()) {
                autoRings++;
            }
        }

        return new AutoCommandGroup(localizationSubsystem, swerveSubsystem, armSubsystem, shooterSubsystem,
                intakeSubsystem, xboxControllerSubsystem, endLoc, shootPose, autoRings,
                autoOptions.get(0).getSelected(), autoOptions.get(1).getSelected(),
                autoOptions.get(2).getSelected(), autoOptions.get(3).getSelected(),
                autoOptions.get(4).getSelected(), autoOptions.get(5).getSelected(),
                autoOptions.get(6).getSelected(), autoOptions.get(7).getSelected());
    }
}
