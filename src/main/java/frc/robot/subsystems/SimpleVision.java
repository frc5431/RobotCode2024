package frc.robot.subsystems;

import java.util.Optional;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AprilTagCamera;
import frc.robot.Constants;
import frc.robot.ObjectCamera;
import frc.robot.TypedApriltag;

public class SimpleVision extends SubsystemBase {
  public AprilTagCamera shooterCamera;
  public ObjectCamera objectCamera;

  public SimpleVision() {
    shooterCamera = new AprilTagCamera(
            "shooter_camera",
            Constants.VisionConstants.SHOOTER_CAMERA_POSE,
            Constants.VisionConstants.ARDUCAM_CAMERA_RESOLUTION,
            Constants.VisionConstants.SHOOTER_CAMERA_FOV);
  }

  
//  public double getXToStage() {
//     Stream<TypedApriltag> targets = shooterCamera.getTargets();

//     targets = targets.filter(target -> target.isStage());

//     Optional<TypedApriltag> finalTarget = targets.findFirst();

//     if(finalTarget.isEmpty()) {
//       // Logger.l("NO VALID TARGETS");
//       return 0;
//     }

//     var cameraToTarget = finalTarget.get().trackedTarget.get().getBestCameraToTarget().plus(new Transform3d(-1, 0, 0, new Rotation3d()));

//     var robotToTarget = cameraToTarget.plus(shooterCamera.getTransform());
    
//     return robotToTarget.getX();
//   }
  
//   public double getYToStage() {
//     Stream<TypedApriltag> targets = shooterCamera.getTargets();

//     // targets = targets.sorted((a, b) -> (int) Math.round(a.trackedTarget.get().getPoseAmbiguity() * 1000) - (int) Math.round(b.trackedTarget.get().getPoseAmbiguity() * 1000));
//     targets = targets.filter(target -> target.isStage());

//     Optional<TypedApriltag> finalTarget = targets.findFirst();

//     if(finalTarget.isEmpty()) {
//       // Logger.l("NO VALID TARGETS");
//       return 0;
//     }

//     var cameraToTarget = finalTarget.get().trackedTarget.get().getBestCameraToTarget().plus(new Transform3d(-1, 0, 0, new Rotation3d()));

//     var robotToTarget = cameraToTarget.plus(shooterCamera.getTransform());
    
//     return robotToTarget.getY();
//   }

  public Optional<Rotation2d> getAngleToSpeaker() {
    Stream<TypedApriltag> targets = shooterCamera.getTargets();

    // targets = targets.sorted((a, b) -> (int) Math.round(a.trackedTarget.get().getPoseAmbiguity() * 1000) - (int) Math.round(b.trackedTarget.get().getPoseAmbiguity() * 1000));
    targets = targets.filter(target -> target.isSpeaker() || target.isStage() && (target.id != 8 || target.id != 5));

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
