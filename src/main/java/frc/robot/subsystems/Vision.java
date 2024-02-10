package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.ApriltagConstants.zone;
import frc.robot.TypedApriltag;

public class Vision {

  protected TypedApriltag apriltag;
  public PhotonCamera apriltagCamera = new PhotonCamera("apriltag_camera");
  public PhotonCamera noteCamera = new PhotonCamera("apriltagn't_camera");

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
  public double getTargetYaw(zone desiredZone, double fallback) {
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

}
