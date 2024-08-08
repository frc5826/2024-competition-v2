package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class ShooterMath {

    public static double getFieldDistance(Pose2d robotPose, Pose2d targetPose) {
        return Math.abs(Math.hypot(robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY()));
    }

    public static double getShootingAngle(Pose2d robotPose, Pose2d targetPose) {
        double fieldDistance = getFieldDistance(robotPose, targetPose);

//        double heightCussion = Math.pow(.03 * fieldDistance, 2) - .12;

//        System.out.println("Height cussion: " + heightCussion);

        double guess =  90 - Math.toDegrees(Math.atan2(Constants.cSpeakerTargetHeight - (Constants.averageArmHeight),
                fieldDistance));
        double armHeight = Math.cos(Math.toRadians(guess)) * .97;
        for(int i = 0; i < 50; i++){
            guess = 90 - Math.toDegrees(Math.atan2(Constants.cSpeakerTargetHeight - armHeight,
                    fieldDistance));
            armHeight = Math.cos(Math.toRadians(guess)) * .97;
        }

        return guess + (Constants.armErrorTolerance * 360);
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

    public static double getSpeakerTurn(Pose2d targetPose, Pose2d currentPose, boolean speaker) {
        return -ShooterMath.fixSpin(targetPose.getTranslation()
                .minus(currentPose.getTranslation()).getAngle().getRadians()
                - currentPose.getRotation().getRadians() - (speaker ? Math.PI : 0));
    }

    public static Rotation2d getAngleToSpeaker(Pose2d targetPose, Pose2d pose) {
        return targetPose.getTranslation()
                .minus(pose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180));
    }

}
