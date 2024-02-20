package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PlacedCamera extends PhotonCamera {
    private final Transform3d location;
    public final PhotonPoseEstimator poseEstimator;

    private PlacedCamera(NetworkTableInstance instance, String cameraName, Transform3d location) {
        super(instance, cameraName);
        this.location = location;
        poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this, location);
    }

    public PlacedCamera(String cameraName, Transform3d location) {
        this(NetworkTableInstance.getDefault(), cameraName, location);
    }

    public Transform3d getPositionOnRobot() {
        return location;
    }
}
