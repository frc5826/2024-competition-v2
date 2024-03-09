package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.cDriveDeadband;
import static frc.robot.Constants.cTurnDeadband;

public class TeleopDriveCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier angleVel;
    private final DoubleSupplier xboxLeft;
    private final DoubleSupplier xboxRight;

    public TeleopDriveCommand(SwerveSubsystem swerveSubsystem,
                              DoubleSupplier x, DoubleSupplier y, DoubleSupplier angleVel,
                              DoubleSupplier xboxLeft, DoubleSupplier xboxRight) {

        this.swerveSubsystem = swerveSubsystem;

        this.xboxLeft = xboxLeft;
        this.xboxRight = xboxRight;

        this.x = x;
        this.y = y;
        this.angleVel = angleVel;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        double x0 = x.getAsDouble();
        double y0 = y.getAsDouble();
        double angleV = angleVel.getAsDouble();

        double bandedx = Math.abs(x0) < cDriveDeadband ? 0 : x0;
        double bandedy = Math.abs(y0) < cDriveDeadband ? 0 : y0;

        double bandedAngle = Math.abs(angleV) < cTurnDeadband ? 0 : angleV;

        ChassisSpeeds speeds = new ChassisSpeeds(bandedx * swerveSubsystem.maximumSpeed,
                bandedy * swerveSubsystem.maximumSpeed,
                bandedAngle * swerveSubsystem.maximumAngularVel);

        if (xboxLeft.getAsDouble() < 0.5 && xboxRight.getAsDouble() < 0.5) {
            swerveSubsystem.driveFieldOriented(speeds);
        } else if (xboxLeft.getAsDouble() > 0.5) {
            swerveSubsystem.driveRobotOriented(new ChassisSpeeds(-0.4, 0, 0));
        } else if(xboxRight.getAsDouble() > 0.5) {
            swerveSubsystem.driveRobotOriented(new ChassisSpeeds(0.4, 0, 0));
        }

    }

}
