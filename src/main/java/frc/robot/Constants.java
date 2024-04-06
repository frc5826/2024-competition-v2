// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
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

    public static final double ringLeftOffset = 0;
    public static final double ringRightOffset = 0;

    public static final double ampArmAngle = 55;
    public static final double intakeArmAngle = -10;
    public static final double homeArmAngle = -10;
    public static final double shootArmAngle = 48;
    public static final double climbArmAngle = 100;
    //public static final double shootArmAngle = 70;

    public static final double maxShooterRPM = 4500;
    public static final double shooterTolerance = maxShooterRPM * 0.07;

    public static final double climbUpPower = 0.25;
    public static final double climbDownPower = -1;

    public static final double armErrorTolerance = 0.0075;

    public static final int climberLeftID = 9;
    public static final int climberRightID = 10;

    public static final int shooterMotor1ID = 13;
    public static final int shooterMotor2ID = 15;
    public static final int intakeID = 14;

    public static final int intakeBeamBreak = 1;
    public static final int shooterBeamBreak = 2;

    public static class ArmConstants {
        public static final int rotateEncoderID = 3;
        public static final int rotateMotor1ID = 11;
        public static final int rotateMotor2ID = 12;
        public static final double armOffset = 0.636;
        public static final double cRotateP = 6;
        public static final double cRotateI = 0;
        public static final double cRotateD = 0;
        public static final double cRotateMax = 1;
        public static final double cRotateMin = -1;
        public static final double cRotateDeadband = 0.0;

        public static final double cGravityConstant = 0.0275;

        public static final double cVelConstant = 0;

        public static final double cMaxVel = 0;

        public static final double cMaxAccel = 0;
    }

    public static final double cDriveDeadband = 0.15;
    public static final double cTurnDeadband = 0.15;

    //shooting constants
    public static final double cSpeakerTargetHeight = 2.15;
    public static final double heightCussion = 0;
    public static final double averageArmHeight = 1;

    public static final PIDConstants cTurnPID = new PIDConstants(3.0, 0.01, 0.25);
    public static final PIDConstants cDrivePID = new PIDConstants(4.0, 0, 0.5);
    public static final double trackRingVel = 3;

    public static final PathConstraints pathConstraints = new PathConstraints(
            3.6, //TODO max this then tune PID
            4.0,
            2 * Math.PI,
            2 * Math.PI);

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
