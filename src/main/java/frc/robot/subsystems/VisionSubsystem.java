// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.AprilTagResult;
import frc.robot.vision.RobotCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.LinkedList;
import java.util.List;


public class VisionSubsystem extends LoggedSubsystem {
    //PhotonLib provides us a "PoseAmbiguity". According to their docs "Numbers above 0.2 are likely to be ambiguous".
    private static final double POSE_CUTOFF = 0.2;

    private final List<RobotCamera> cameras;

    private RobotCamera frontCamera;

    public VisionSubsystem() {
        frontCamera = new RobotCamera(new Translation3d(.3225,-.28,.225), new Rotation3d(0,0,0), "beta-3000", false);

        cameras = List.of(
                new RobotCamera(new Translation3d(.2425, -.3075,.27), new Rotation3d(Math.PI,Math.PI / 12,0), "beta-studio", true),
                frontCamera,
                new RobotCamera(new Translation3d(.3025,.2575,.27), new Rotation3d(Math.PI,Math.PI / 12, Math.PI), "gamma-studio", true),
                new RobotCamera(new Translation3d(-.3225,.3075,.225), new Rotation3d(0,0,Math.PI), "gamma-3000", false)
        );

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

    public RobotCamera getFrontCamera() {
        return frontCamera;
    }

}