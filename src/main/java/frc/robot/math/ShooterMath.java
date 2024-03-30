package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class ShooterMath {

    public static double getFieldDistance(Pose2d robotPose, Pose2d targetPose) {
        return Math.abs(Math.hypot(robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY()));
    }

    public static double getShootingAngle(Pose2d robotPose, Pose2d targetPose) {
//        return 90 - Math.toDegrees(Math.atan2(Constants.cSpeakerTargetHeight + Constants.heightCussion - (Constants.averageArmHeight),
//                getFieldDistance(robotPose, targetPose)));
        double guess =  90 - Math.toDegrees(Math.atan2(Constants.cSpeakerTargetHeight + Constants.heightCussion - (Constants.averageArmHeight),
                getFieldDistance(robotPose, targetPose)));
        double armHeight = Math.cos(Math.toRadians(guess)) * .97;
        for(int i = 0; i < 10; i++){
            guess = 90 - Math.toDegrees(Math.atan2(Constants.cSpeakerTargetHeight + Constants.heightCussion - (armHeight),
                    getFieldDistance(robotPose, targetPose)));
            armHeight = Math.cos(Math.toRadians(guess)) * .97;
        }

        return guess;
    }

    //when getting a difference between two angles this method will
    // allow the angle to overflow correctly for turning with swerve drive
    public static double fixSpin(double angleDifference) {
        if(angleDifference < -Math.PI){
            angleDifference += 2*Math.PI;
        } else if (angleDifference > Math.PI) {
            angleDifference -= 2*Math.PI;
        }

        return angleDifference;
    }

}
