package frc.robot.subsystems;

import java.util.Optional;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagCamera;
import frc.robot.Constants;
import frc.robot.TypedApriltag;

public class SimpleVision extends SubsystemBase {
  public AprilTagCamera shooterCamera;

  public SimpleVision() {
    shooterCamera = new AprilTagCamera(
            "shooter_camera",
            Constants.VisionConstants.SHOOTER_CAMERA_POSE,
            Constants.VisionConstants.ARDUCAM_CAMERA_RESOLUTION,
            Constants.VisionConstants.SHOOTER_CAMERA_FOV);

  }

  public Optional<Rotation2d> getAngleTowardsStage() {
    Stream<TypedApriltag> targets = shooterCamera.getTargets();

    targets = targets.sorted((a, b) -> (int) Math.round(a.trackedTarget.get().getPoseAmbiguity() * 1000) - (int) Math.round(b.trackedTarget.get().getPoseAmbiguity() * 1000));
    targets = targets.filter(target -> target.isFriendlyApriltag() && target.isSpeaker() || target.isStage());

    if(targets.count() == 0) {
      return Optional.empty();
    }

    TypedApriltag finalTarget = targets.toList().get(0);

    var cameraToTarget = finalTarget.trackedTarget.get().getBestCameraToTarget();

    var robotToTarget = cameraToTarget.plus(shooterCamera.getTransform());

    
    return Optional.of(Rotation2d.fromRadians(Math.atan2(robotToTarget.getY(), robotToTarget.getX())));
  }
}
