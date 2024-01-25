package frc.robot;

import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
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

  public static class IntakeConstants {
    public static double intakePower = .5;
    public static double outtakePower = -.5;

  }

    public static class AnglerConstants {
    public static Rotation2d retractAngle = Rotation2d.fromDegrees(0);
    public static Rotation2d deployAngle = Rotation2d.fromDegrees(105);
    public static double p = .5;
    public static double i = .1;
    public static double d = .5;
  }


  public static final int ID_PIGEON2 = 13;
    public static final int ID_PHUB = 1;

    public static final String CANBUS_DRIVETRAIN = "Omnivore2024"; // "omnivore"
    public static final String CANBUS_SUBSYSTEM = "";

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.546;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.648;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.434082; // 293.906

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.413086; // 105.381

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.070068; // 111.533

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -1.077881; // 53.701


}


