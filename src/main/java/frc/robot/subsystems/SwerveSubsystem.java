package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;

public class SwerveSubsystem extends LoggedSubsystem {
    private final SwerveDrive swerveDrive;

    public double maximumSpeed = Constants.maxVelocity;

    public double maximumAngularVel = 5;

    public SwerveSubsystem(File directory) {

        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(
                12.8, 1);

        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
                Units.inchesToMeters(4), 8.14, 1);

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.NONE;
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void stop() {
        driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }

    public double getDriveBaseRadius() {
        return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
    }

    public void driveRobotOriented(ChassisSpeeds velocity)
    {
        swerveDrive.drive(velocity);
    }

    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
    }

    public void setGyro(Rotation3d rotation){
        swerveDrive.setGyro(rotation);
    }

    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    public void zeroOdometry() {
        swerveDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    public Rotation2d getHeading()
    {
        return swerveDrive.getYaw();
    }

    public ChassisSpeeds getFieldVelocity()
    {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    public Rotation2d getGyroRotation() {
        return getHeading();
    }

    public SwerveModulePosition[] getModulePositions() {
        return swerveDrive.getModulePositions();
    }

}
