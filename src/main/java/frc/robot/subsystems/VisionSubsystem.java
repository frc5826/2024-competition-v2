// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.RingMath;
import frc.robot.vision.AprilTagResult;
import frc.robot.vision.RingResult;
import frc.robot.vision.RobotCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.LinkedList;
import java.util.List;


public class VisionSubsystem extends LoggedSubsystem {
    //PhotonLib provides us a "PoseAmbiguity". According to their docs "Numbers above 0.2 are likely to be ambiguous".
    private static final double POSE_CUTOFF = 0.2;

    private final List<RobotCamera> cameras;

    public VisionSubsystem() {
        //Camera position conventions:
        //  +x means camera is on the front of the robot
        //  -x means camera is on the back of robot
        //  +y means camera is on the left of the robot
        //  -y means camera is on the right of the robot
        //  +angles are counterclockwise
        
        cameras = List.of(
//                new RobotCamera(new Translation3d(inToMeters(2),inToMeters(-9.5),inToMeters(22)), new Rotation3d(0,Math.PI / 12,0), "beta-studio-1", true),
//                new RobotCamera(new Translation3d(inToMeters(1),inToMeters(-13),inToMeters(21.5)), new Rotation3d(0,-Math.PI / 6,0), "beta-studio-2", true),
                //new RobotCamera(new Translation3d(inToMeters(-2),inToMeters(9.5),inToMeters(22)), new Rotation3d(0,Math.PI / 12, 0), "gamma-studio", true),
                new RobotCamera(new Translation3d(inToMeters(1),inToMeters(13),inToMeters(21.5)), new Rotation3d(0,-Math.PI / 6, 0), "gamma-3000", false),
                //new RobotCamera(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0), "alpha-studio", true),
                new RobotCamera(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0), "alpha-3000", false)
        );

    }


    private double inToMeters(double in) {
        return in / 39.37;
    }


    public List<AprilTagResult> getAprilTagResults(){
        List<AprilTagResult> output = new LinkedList<>();

        for(RobotCamera camera : cameras){
            if(camera.isAprilTag()){
                PhotonPipelineResult result = camera.getCamera().getLatestResult();
                if(result.hasTargets()){
                    List<PhotonTrackedTarget> targets = result.getTargets();
                    for(PhotonTrackedTarget target : targets) {
                        if(target.getFiducialId() > -1 && target.getPoseAmbiguity() <= POSE_CUTOFF && target.getPoseAmbiguity() != -1) {
                            output.add(new AprilTagResult(camera, target.getBestCameraToTarget(), target.getFiducialId(), target.getPoseAmbiguity(), result.getTimestampSeconds()));
                        }
                    }
                }
            }
        }

        return output;
    }

    public List<Pair<PhotonTrackedTarget, RobotCamera>> getRings() {
        List<Pair<PhotonTrackedTarget, RobotCamera>> targets = new LinkedList<>();

        for (RobotCamera camera : cameras) {
            if (!camera.isAprilTag()) {
                PhotonPipelineResult result = camera.getCamera().getLatestResult();
                if (result.hasTargets()) {
                    for (PhotonTrackedTarget target : result.getTargets()) {
                        if (target.getPitch() + Math.toDegrees(camera.getCameraPostion().getRotation().getY()) < 0) {
                            targets.add(new Pair<>(target, camera));
                        }
                    }
                }
            }
        }

        return targets;
    }

    public Pair<RobotCamera, List<PhotonTrackedTarget>> getRings(boolean isLeft) {
        List<PhotonTrackedTarget> targets = new LinkedList<>();

        RobotCamera camera = isLeft ? cameras.get(0) : cameras.get(1);

        if (!camera.isAprilTag()) {
            PhotonPipelineResult result = camera.getCamera().getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    targets.add(target);
                }
            }
        }

        return new Pair<>(camera, targets);
    }

}
