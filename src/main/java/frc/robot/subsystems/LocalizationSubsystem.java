package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.AprilTagResult;
import frc.robot.vision.RingResult;
import frc.robot.vision.RobotCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.*;

import static frc.robot.positioning.FieldOrientation.getOrientation;

//Code pulled from - https://github.com/STMARobotics/frc-7028-2023/blob/5916bb426b97f10e17d9dfd5ec6c3b6fda49a7ce/src/main/java/frc/robot/subsystems/PoseEstimatorSubsystem.java
public class LocalizationSubsystem extends SubsystemBase {

    private AprilTagFieldLayout fieldLayout;
    private final VisionSubsystem visionSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final HashMap<Integer, AprilTagResult> processed;
    private Pose3d robotPos = new Pose3d();

    private final Field2d field = new Field2d();

    private Rotation2d rotationTarget;

    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(10));

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(5));

    private List<RingResult> ringResultsLeft;
    private List<RingResult> ringResultsRight;

    private RingResult bestFrontRing = RingResult.getEmpty();

    private RingResult bestLeftRing = RingResult.getEmpty();
    private RingResult bestRightRing = RingResult.getEmpty();

    private long lastTimeRingSeen;

    public LocalizationSubsystem(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem) {
        try {
            //We're always going to use the "blue wall" as or origin. All positions will be absolute.
            //We'll use the FieldOrientation class to get us the right position based on our alliance.
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            fieldLayout = null;
            e.printStackTrace();
        }
        this.processed = new HashMap<>();

        this.visionSubsystem = visionSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        //TODO - Is there a better guess at initial pose?
        this.poseEstimator = new SwerveDrivePoseEstimator(swerveSubsystem.getKinematics(), swerveSubsystem.getGyroRotation(), swerveSubsystem.getModulePositions(), new Pose2d(), stateStdDevs, visionMeasurementStdDevs);

        setupShuffleboard(field);

        PPHolonomicDriveController.setRotationTargetOverride(this::updateRotationTarget);

        setupPathPlanner();

        lastTimeRingSeen = 0;
    }

    public void setRotationTarget(Rotation2d rotation) { rotationTarget = rotation; }

    public Rotation2d getRotationTarget() { return rotationTarget; }

    public void removeRotationTarget() { rotationTarget = null; }

    private Optional<Rotation2d> updateRotationTarget() {
        if (rotationTarget != null) {
            return Optional.of(rotationTarget);
        } else {
            return Optional.empty();
        }
    }

    public void periodic() {


        if(fieldLayout != null){
            for(AprilTagResult result : visionSubsystem.getAprilTagResults()){
                if(!processed.containsKey(result.getId()) || processed.get(result.getId()).getTimestamp() != result.getTimestamp()) {
                    processed.put(result.getId(), result);
                    Optional<Pose3d> tagPose = fieldLayout.getTagPose(result.getId());
                    if (tagPose.isPresent()) {
                        Pose3d camPose = tagPose.get().transformBy(result.getAprilTagLocation().inverse());
                        Pose3d robotPose = camPose.transformBy(result.getCamera().getRobotLocation());
                        robotPos = robotPose;

                        poseEstimator.addVisionMeasurement(robotPose.toPose2d(), result.getTimestamp());
                    }
                }
            }

            poseEstimator.update(swerveSubsystem.getGyroRotation(), swerveSubsystem.getModulePositions());
        }
        else {
            System.err.println("Unable to localize. Field Layout not loaded.");
        }
        field.setRobotPose(getCurrentPose());
        //field.getObject("rings").setPose(new Pose2d(bestFrontRing.getFieldPose(), Rotation2d.fromDegrees(0)));

        ringResultsLeft = getRingResults(visionSubsystem.getRings(true), true);
        ringResultsRight = getRingResults(visionSubsystem.getRings(false), false);

        bestLeftRing = getBestPickupRing(ringResultsLeft);
        bestRightRing = getBestPickupRing(ringResultsRight);

//        field.getObject("rings").setPoses(
//                new Pose2d(bestLeftRing.getFieldPose(), new Rotation2d()),
//                new Pose2d(bestRightRing.getFieldPose(), new Rotation2d()));

        if (seesRing()){
            lastTimeRingSeen = System.currentTimeMillis();
        }
    }

    public RingResult getBestLeftRing() {
        return bestLeftRing;
    }

    public RingResult getBestRightRing() {
        return bestRightRing;
    }

    public void reset() {
        poseEstimator.resetPosition(
                swerveSubsystem.getGyroRotation(),
                swerveSubsystem.getModulePositions(),
                new Pose2d(0,0,Rotation2d.fromRadians(0)));
    }

    public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
                this::getCurrentPose, // Robot pose supplier
                swerveSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveSubsystem::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerveSubsystem::driveRobotOriented, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        Constants.cDrivePID,
                        // Translation PID constants
                        //swerveSubsystem.getPID(),
                        Constants.cTurnPID,
                        // Rotation PID constants
                        Constants.maxVelocity,
                        // Max module speed, in m/s
                        swerveSubsystem.getDriveBaseRadius(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                        // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                swerveSubsystem // Reference to this subsystem to set requirements
        );
    }

    public Command buildPath(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                3.1,
                4,
                2 * Math.PI,
                3 * Math.PI);

        Command path = AutoBuilder.pathfindToPose(targetPose, constraints);

        return path;
    }

    public Command buildPathThenFollow(Pose2d endPose, Pose2d startPose) {
        List<Translation2d> points = PathPlannerPath.bezierFromPoses(startPose, endPose);

        PathPlannerPath path = new PathPlannerPath(points , Constants.pathConstraints, new GoalEndState(0, endPose.getRotation(), true));
        path.preventFlipping = true;

        return AutoBuilder.pathfindThenFollowPath(path, Constants.pathConstraints);
    }

    private void setupShuffleboard(Field2d field) {
        ShuffleboardTab tab = Shuffleboard.getTab("position");

        tab.add(field)
                .withPosition(2,0)
                .withSize(5,3);

        ShuffleboardLayout position = tab.getLayout("Robot position", BuiltInLayouts.kList)
                .withPosition(0,0)
                .withSize(2,2);


//        position.addDouble("Robot X", ()-> getCurrentPose().getX());
//        position.addDouble("Robot Y", ()-> getCurrentPose().getY());
//        position.addDouble("Robot rotation", ()-> swerveSubsystem.getHeading().getDegrees());


        ShuffleboardLayout robot3DPose = tab.getLayout("robot 3d pose", BuiltInLayouts.kList)
                .withPosition(7, 0)
                .withSize(2, 4);

        tab.add("Reorient Gyro", new InstantCommand(() -> {
                    if(getOrientation().isValid()){
                        swerveSubsystem.setGyro(new Rotation3d(0, 0, getCurrentPose().getRotation().getRadians() + (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? Math.PI : 0 )));
                    }
                }))
                .withSize(2, 1)
                .withPosition(0,3);

        robot3DPose.addDouble("x", ()-> robotPos.getX());
        robot3DPose.addDouble("y", ()-> robotPos.getY());
        robot3DPose.addDouble("z", ()-> robotPos.getZ());
        robot3DPose.addDouble("roll", ()-> Math.toDegrees(robotPos.getRotation().getX()));
        robot3DPose.addDouble("pitch", ()-> Math.toDegrees(robotPos.getRotation().getY()));
        robot3DPose.addDouble("yaw", ()-> Math.toDegrees(robotPos.getRotation().getZ()));

        ShuffleboardTab testTab = Shuffleboard.getTab("test tab");

        testTab.addBoolean("SeesRing",this::seesRing);

//        testTab.addDouble("left ring distance", () -> bestLeftRing.getDistance());
//        testTab.addDouble("left ring yaw", () -> Math.toDegrees(bestLeftRing.getAngleToHeading()));
//        testTab.addDoubleArray("left ring field pose", () -> new double[]{bestLeftRing.getFieldPose().getX(), bestLeftRing.getFieldPose().getY()});
//
//        testTab.addDouble("right ring distance", () -> bestRightRing.getDistance());
//        testTab.addDouble("right ring yaw", () -> Math.toDegrees(bestRightRing.getAngleToHeading()));
//        testTab.addDoubleArray("right ring field pose", () -> new double[]{bestRightRing.getFieldPose().getX(), bestRightRing.getFieldPose().getY()});
        testTab.addDouble("Time Since Seen Ring", this::timeSinceSeenRing);
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public List<RingResult> getRingResults(Pair<RobotCamera, List<PhotonTrackedTarget>> rings, boolean isLeft) {
        List<RingResult> ringResults = new LinkedList<>();

        for (PhotonTrackedTarget ring : rings.getSecond()) {
            ringResults.add(new RingResult(rings.getFirst(),
                    ring.getYaw(),
                    ring.getPitch(),
                    ring.getArea(),
                    getCurrentPose(),
                    isLeft));
        }

        return ringResults;
    }

    public RingResult getBestPickupRing(List<RingResult> rings) {
        RingResult ring = RingResult.getEmpty();

        for (RingResult ringResult : rings) {
            if (ringResult.getPitch() < ring.getPitch()){
                ring = ringResult;
            }
        }

        return ring;
    }

    public boolean seesRing() {
        return !bestLeftRing.equals(RingResult.getEmpty()) ||
                !bestRightRing.equals(RingResult.getEmpty());
    }

    public double timeSinceSeenRing(){
        return ((double)(System.currentTimeMillis() - lastTimeRingSeen)) / 1000;
    }
}
