package frc.robot.subsystems;

import java.util.Optional;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagCamera;
import frc.robot.Constants;
import frc.robot.TypedApriltag;
import frc.team5431.titan.core.misc.Logger;

public class SimpleVision extends SubsystemBase {
  public AprilTagCamera shooterCamera;

  public SimpleVision() {
    shooterCamera = new AprilTagCamera(
            "shooter_camera",
            Constants.VisionConstants.SHOOTER_CAMERA_POSE,
            Constants.VisionConstants.ARDUCAM_CAMERA_RESOLUTION,
            Constants.VisionConstants.SHOOTER_CAMERA_FOV);
  }


  public Optional<Rotation2d> getAngleToSpeaker() {
    Stream<TypedApriltag> targets = shooterCamera.getTargets();

    // targets = targets.sorted((a, b) -> (int) Math.round(a.trackedTarget.get().getPoseAmbiguity() * 1000) - (int) Math.round(b.trackedTarget.get().getPoseAmbiguity() * 1000));
    targets = targets.filter(target -> target.isSpeaker() && (target.id != 8 || target.id != 5));

    Optional<TypedApriltag> finalTarget = targets.findFirst();

    if(finalTarget.isEmpty()) {
      // Logger.l("NO VALID TARGETS");
      return Optional.empty();
    }

    var cameraToTarget = finalTarget.get().trackedTarget.get().getBestCameraToTarget().plus(new Transform3d(-1, 0, 0, new Rotation3d()));

    var robotToTarget = cameraToTarget.plus(shooterCamera.getTransform());

    var targetRads = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
    
    return Optional.of(Rotation2d.fromRadians(-targetRads));
  }
}
