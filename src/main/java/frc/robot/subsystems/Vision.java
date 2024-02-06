package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.ApriltagConstants.zone;
import frc.robot.TypedApriltag;


public class Vision {

  protected TypedApriltag apriltag;
  public PhotonCamera apriltagCamera = new PhotonCamera("apriltag_camera");
  public PhotonCamera noteCamera = new PhotonCamera("apriltagn't_camera");
  public AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public void updateEstimatedPose(PhotonPoseEstimator estimator) {
    var result = apriltagCamera.getLatestResult();
    estimator.update(result);
  }

  
  /**
   * Gives the yaw of apriltag for locking onto an apriltag
   * 
   * @param desiredZone speaker, amp, stage, or source
   * @param fallback rotation based on controller
   * @return yaw of apriltag target, or fallback
   */
  public double getTargetYawZone(zone desiredZone, double fallback) {
    var result = apriltagCamera.getLatestResult();
    //https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#checking-for-existence-of-targets
    boolean live = result.hasTargets();
    PhotonTrackedTarget target = result.getBestTarget();

    int id = target.getFiducialId();
    TypedApriltag apriltag = new TypedApriltag(id);
    zone detect = apriltag.zoneMatch();

    if (apriltag.isFriendlyApriltag() && detect == desiredZone) {
      return target.getYaw();
    }
    return fallback;
  }

  public double getTargetYaw() {
    var result = apriltagCamera.getLatestResult();
    boolean live = result.hasTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    return target.getYaw();
  }

  // public double getTargetDistance(Pose2d robotPose2d) {
  //   var result = apriltagCamera.getLatestResult();
  //   boolean live = result.hasTargets();
  //   PhotonTrackedTarget target = result.getBestTarget();
  //   Optional<Pose3d> targetPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
  //   targetPose.
  //   double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose2d, targetPose);


  //   return 2;
  // }

}
