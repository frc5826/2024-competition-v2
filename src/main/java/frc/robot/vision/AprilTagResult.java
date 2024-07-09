package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.Objects;

public class AprilTagResult {

    private final Transform3d aprilTagLocation;
    private final int id;
    private final double ambiguity;
    private final RobotCamera camera;
    private final double timestamp;

    public AprilTagResult(RobotCamera camera, Transform3d aprilTagLocation, int id, double ambiguity, double timestamp) {
        this.camera = camera;
        this.aprilTagLocation = aprilTagLocation;
        this.id = id;
        this.ambiguity = ambiguity;
        this.timestamp = timestamp;
    }

    public Transform3d getAprilTagLocation() {
        return aprilTagLocation;
    }

    public Transform3d getAprilTagLocation(boolean robotEnabled, AprilTagFieldLayout fieldLayout, Rotation2d gyro) {
        if (robotEnabled) {
            return new Transform3d(aprilTagLocation.getTranslation(),
                    new Rotation3d(aprilTagLocation.getRotation().getX(), aprilTagLocation.getRotation().getY(),
                            gyro.plus(new Rotation2d(camera.getCameraPostion().getRotation().getZ()))
                                    .plus(fieldLayout.getTags().get(id).pose.getRotation().toRotation2d()).getRadians()));
        } else {
            return aprilTagLocation;
        }
    }

    public int getId() {
        return id;
    }

    public double getAmbiguity() {
        return ambiguity;
    }

    public RobotCamera getCamera() {
        return camera;
    }

    public double getTimestamp() {
        return timestamp;
    }

    @Override
    public String toString() {
        return "AprilTagResult{" +
                "aprilTagLocation=" + aprilTagLocation +
                ", id=" + id +
                ", ambiguity=" + ambiguity +
                ", camera=" + camera +
                '}';
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        AprilTagResult that = (AprilTagResult) o;

        if (id != that.id) return false;
        if (Double.compare(timestamp, that.timestamp) != 0) return false;
        return Objects.equals(camera, that.camera);
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = id;
        result = 31 * result + (camera != null ? camera.hashCode() : 0);
        temp = Double.doubleToLongBits(timestamp);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
