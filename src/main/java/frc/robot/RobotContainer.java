// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.TeleopDriveCommand;
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

    private final LocalizationSubsystem localizationSubsystem = new LocalizationSubsystem(visionSubsystem, swerveSubsystem);

    private final TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(
            swerveSubsystem,
            ()-> -xbox.getLeftY(), ()-> -xbox.getLeftX(), ()-> -xbox.getRightX(),
            ()-> xbox.getLeftTriggerAxis(), ()-> xbox.getRightTriggerAxis());

    public RobotContainer()
    {
        setupButtonBoardBindings();
        setupXboxBindings();
        SignalLogger.start();

        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, teleopDriveCommand);
    }

    private void setupXboxBindings() {

    }

    private void setupButtonBoardBindings() {

    }

//    public Command getAutonomousCommand()
//    {
//
//    }
}
