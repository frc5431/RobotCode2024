package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ApriltagConstants.zone;
import frc.robot.PlacedCamera;
import frc.robot.Systems;
import frc.robot.TimedThreadExecutor;
import frc.robot.TypedApriltag;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

public class Vision extends SubsystemBase {

  protected TypedApriltag apriltag;
  private List<PlacedCamera> cameras;
  public PheonixDrivebase drivebase;
  public PhotonPoseEstimator poseEstimator;
  public Field2d field;
  public TimedThreadExecutor tte = new TimedThreadExecutor(() -> {
    var poseEstimator = drivebase.getOdometry();
    this.updateEstimatedPose(poseEstimator);
    field.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field", field);
  }, 100, TimeUnit.MILLISECONDS);

  public Vision(Systems systems, PlacedCamera... cameras) {
    this.cameras = Arrays.stream(cameras).toList();
    poseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
        PoseStrategy.LOWEST_AMBIGUITY, new Transform3d());
    this.drivebase = systems.getDrivebase();
    field = new Field2d();
  }

  public void updateEstimatedPose(SwerveDrivePoseEstimator estimator) {
    Pose2d prevEstimatedRobotPose = estimator.getEstimatedPosition();

    for (int i = 0; i < cameras.size(); i++) {
      var cam = cameras.get(i);
      cam.poseEstimator.setReferencePose(prevEstimatedRobotPose);
      Optional<EstimatedRobotPose> est = cam.poseEstimator.update();
      if (est.isPresent()) {
        var estimate = est.get();
        // System.out.println("Present");
        try {
          estimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
        } catch (Exception ignored) {
          // System.out.println("Errored!");
        }
      }
    }
  }

  /**
   * Gives the yaw of apriltag for locking onto an apriltag
   * 
   * @param desiredZone speaker, amp, stage, or source
   * @param fallback    rotation based on controller
   * @return yaw of apriltag target, or fallback
   */
  public double getTargetYaw(zone desiredZone, double fallback, PlacedCamera apriltagCamera) {
    var result = apriltagCamera.getLatestResult();
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#checking-for-existence-of-targets
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

  @Override
  public void periodic() {
    try {
      // tte.run();
    }catch(OutOfMemoryError ignored) {
     // System.out.println("out of memory issue with thread");
    }catch(Exception err) {
      err.printStackTrace();
    }
  }

}
