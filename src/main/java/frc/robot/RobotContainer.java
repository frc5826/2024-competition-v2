// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.arm.ArmCommand;
import frc.robot.commands.arm.TargetSpeakerCommand;
import frc.robot.commands.climb.ClimberCommand;
import frc.robot.commands.drive.AutoDriveToRingCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommandGroup;
import frc.robot.commands.intake.IntakeSecondHalfCommandGroup;
import frc.robot.commands.led.FlashLEDCommand;
import frc.robot.commands.shoot.ShootSpeakerCommandGroup;
import frc.robot.led.TeensyLED;
import frc.robot.subsystems.*;

import java.io.File;

import static frc.robot.Constants.*;


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

    public RobotContainer()
    {
        setupButtonBoardBindings();
        setupXboxBindings();
        SignalLogger.enableAutoLogging(false);

        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, teleopDriveCommand);

        DataLogManager.start();
    }

    private void setupXboxBindings() {

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
        panelButtons[7].onTrue(new ArmCommand(armSubsystem, shootArmAngle));
        panelButtons[8].whileTrue(new IntakeCommand(intakeSubsystem, 0.3));
        panelButtons[9].onTrue(new ArmCommand(armSubsystem, ampArmAngle));
        panelButtons[10].onTrue(new ArmCommand(armSubsystem, homeArmAngle));
        panelButtons[11].onTrue(new ShootSpeakerCommandGroup(shooterSubsystem, intakeSubsystem, armSubsystem, localizationSubsystem, swerveSubsystem));
        //panelButtons[11].onTrue(new RunCommand(() -> shooterSubsystem.setShooterSpeed(500)));
    }

//    public Command getAutonomousCommand()
//    {
//
//    }
}
