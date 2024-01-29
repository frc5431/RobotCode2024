package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.team5431.titan.core.robot.MotionMagic;

public final class Constants {

  public static double vortexStallTorque = 3.6;
  public static double neoStallTorque = 3;
  public static double neo550StallTorque = 0.97;

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class idsorsmth {

    public static final int idorsmth = 0;
  }

  public static class RobotConstants {

    public static Transform3d cameraOffsetPosition = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  }

  public static class ApriltagConstants {

    public enum zone {
      SOURCE,
      AMP,
      SPEAKER,
      STAGE
    }

    // Team
    public static List<Integer> blue = List.of(6, 7, 8, 9, 10, 14, 15, 16);
    public static List<Integer> red = List.of(1, 2, 3, 4, 5, 11, 12, 13);

    // Retreival
    public static List<Integer> source = List.of(9, 10, 1, 2);

    // Scoring
    public static List<Integer> amp = List.of(5, 6);
    public static List<Integer> speaker = List.of(7, 8, 3, 4);

    // well this one's obvious
    public static List<Integer> stage = List.of(11, 12, 13, 14, 15, 16);
  }

  public static class IntakeConstants {

    public static double intakePower = 1;
    public static double outtakePower = -1;
    public static int anglerId = 14;
    public static int intakeId = 16;
    public static AnglerConstants anglerConstants = new AnglerConstants(
      /* Min Angle */Rotation2d.fromRadians(-45), // temp
      /* Max Angle */Rotation2d.fromRadians(115), // temp
      /* Length Meters */Units.inchesToMeters(16),
      /* Weight Kilos */Units.lbsToKilograms(4.01), // temp
      /* Parallel To Ground Angle */Rotation2d.fromDegrees(0),
      /* PID */new MotionMagic(0.5, 0.0, 0.1, -1), // ff goes unused
      /* Stall Torque (Nm) */ neoStallTorque,
      /* Enable FF */ false
    );
  }

  public static class ShooterConstants {

    public static double normalPower = 0.5;
    public static int anglerId = 15; // temp
    public static AnglerConstants anglerConstants = new AnglerConstants(
      /* Min Angle */Rotation2d.fromDegrees(0), // temp
      /* Max Angle */Rotation2d.fromDegrees(90), // temp
      /* Length Meters */Units.inchesToMeters(16), // temp
      /* Weight Kilos */Units.lbsToKilograms(10), // temp
      /* Parallel To Ground Angle */Rotation2d.fromDegrees(0), // temp
      /* PID */new MotionMagic(0.0, 0.0, 0.1, -1), // ff goes unused
      /* Stall Torque (Nm) */ vortexStallTorque,
      /* Enable FF */ true
    );
  }

  public static class AnglerConstants {

    public final Rotation2d minAngle;
    public final Rotation2d maxAngle;
    public final double lengthMeters;
    public final double weight;
    public final Rotation2d parallelToGroundAngle;
    public final MotionMagic pid;
    public final double stalltorque;
    public final boolean enableFF;

    public AnglerConstants(
      Rotation2d minAngle,
      Rotation2d maxAngle,
      double lengthMeters,
      double weight,
      Rotation2d parallelToGroundAngle,
      MotionMagic pid,
      double stallTorque,
      boolean enableFF
    ) {
      this.minAngle = minAngle;
      this.maxAngle = maxAngle;
      this.lengthMeters = lengthMeters;
      this.weight = weight;
      this.parallelToGroundAngle = parallelToGroundAngle;
      this.pid = pid;
      this.stalltorque = stallTorque;
      this.enableFF = enableFF;
    }
  }

  public static class DrivebaseConstant {

    public static final int ID_PIGEON2 = 13;
    public static final int ID_PHUB = 1;

    public static final String CANBUS_DRIVETRAIN = "Omnivore2024"; // "omnivore"
    public static final String CANBUS_SUBSYSTEM = "";

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    // TODO: update this
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.546;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    // TODO: update this
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.648;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
    // TODO: update this
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.434082;

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    // TODO: update this
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.413086;

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    // TODO: update this
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.070068;

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    // TODO: update this
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -1.077881;
  }
}
