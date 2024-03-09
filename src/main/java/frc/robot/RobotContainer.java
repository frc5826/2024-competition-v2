// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import static frc.robot.Constants.*;


public class RobotContainer
{

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory() + "/swerve"));

    private final TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(
            swerveSubsystem,
            ()-> -xbox.getLeftY(), ()-> -xbox.getLeftX(), ()-> -xbox.getRightX(),
            ()-> xbox.getLeftTriggerAxis(), ()-> xbox.getRightTriggerAxis());

    public RobotContainer()
    {
        CommandScheduler.getInstance().setDefaultCommand(swerveSubsystem, teleopDriveCommand);
    }

//    public Command getAutonomousCommand()
//    {
//
//    }
}
