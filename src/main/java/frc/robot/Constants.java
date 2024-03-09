// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final int rotateMotor1ID = 10;
    public static final int rotateMotor2ID = 11;
    public static final int extendMotorID = 14;
    public static final int ankleMotorID = 12;

    public static final int shooterMotor1ID = 3;
    public static final int shooterMotor2ID = 6;
    public static final int shooterControlMotorID = 30;

    public static final int rotateEncoderID = 0;
    public static final int extendEncoderID = 1;
    public static final int ankleEncoderID = 2;

    public static final int beamBreakID = 3;

    public static final double cRotateP = 4.5;
    public static final double cRotateI = 1;
    public static final double cRotateD = 0.2;
    public static final double cRotateMax = 1;
    public static final double cRotateMin = -1;
    public static final double cRotateDeadband = 0;
    public static final double cExtendP = 3.5;
    public static final double cExtendI = 0.2;
    public static final double cExtendD = 0.1;
    public static final double cExtendMax = 1;
    public static final double cExtendMin = -1;
    public static final double cExtendDeadband = 0;

    public static final double cWristP = 4;
    public static final double cWristI = 0.1;
    public static final double cWristD = 0;
    public static final double cWristMax = 1;
    public static final double cWristMin = -1;
    public static final double cWristDeadband = 0;

    public static final double cDriveDeadband = 0.15;
    public static final double cTurnDeadband = 0.15;

    public static final double cSpeakerTargetHeight = 0;
    public static final double cMotorVeltoDistance = 100;
    public static final double cMotorVeltoExitVel = 1;

    public static final PIDConstants cTurnPID = new PIDConstants(4.0, 0.01, 0.25);
    public static final PIDConstants cDrivePID = new PIDConstants(4.0, 0, 0.5);

    public static final double maxVelocity = 3.6; //doesn't make robot faster
    public static final Joystick buttonPanel = new Joystick(2);
    public static final XboxController xbox = new XboxController(1);
    public static final Trigger[] panelButtons;

    static {
        panelButtons = new Trigger[12];

        for(int i = 0; i < panelButtons.length; i++) {
            int finalI = i+1;
            panelButtons[i] = new Trigger(() -> buttonPanel.getRawButton(finalI));
        }
    }
}
