package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  
  }

  public static class idsorsmth {
    public static final int idorsmth = 0;
  
  }

  public static class RobotConstants {
    public static Transform3d cameraOffsetPosition = new Transform3d(
      new Translation3d(0,0,0),
      new Rotation3d(0,0,0)
    );
  }

  public static class ApriltagConstants {
    // Team
    public static List<Integer> blue = List.of(6,7,8,9,10,14,15,16);
    public static List<Integer> red = List.of(1,2,3,4,5,11,12,13);

    // Retreival
    public static List<Integer> source = List.of(9,10,1,2);

    // Scoring
    public static List<Integer> amp = List.of(5,6);
    public static List<Integer> speaker = List.of(7,8,3,4);

    // well this one's obvious
    public static List<Integer> stage = List.of(11,12,13,14,15,16);
  }
}
