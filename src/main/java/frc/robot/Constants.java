package frc.robot;

import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.AprilTagCamera.Resolution;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.MotorRatio;
import frc.team5431.titan.core.robot.MotionMagic;

public final class Constants {
  public static final boolean useXboxController = true; 
  public static double vortexStallTorque = 3.6;
  public static double neoStallTorque = 2.6;
  public static double neo550StallTorque = 0.97;
  public static class idsorsmth {

    public static final int idorsmth = 0;
    public static final boolean isTauseefCool = false;
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
    public static int leftIntakeId = 15;
    public static int rightIntakeId = 16;
    public static Rotation2d ampAngle = Rotation2d.fromDegrees(88);

    public static AnglerConstants anglerConstants = new AnglerConstants(
      /* Min Angle */Rotation2d.fromDegrees(-25),
      /* Max Angle */Rotation2d.fromDegrees(150), // temp
      /* Length Meters */Units.inchesToMeters(12),
      /* Weight Kilos */Units.lbsToKilograms(5.625), // temp
      /* Parallel To Ground Angle */Rotation2d.fromRadians(0),
      /* PID */new MotionMagic(0.3, 0.0, 0.02, -1),
      /* Stall Torque (Nm) */ neoStallTorque * (15),
      /* Enable FF */ true,
      /* Gear Ratio */2,
      0.8
    );

    public static ManipulatorConstants manipulatorConstants = new ManipulatorConstants(
      /* Is Inverted */ true,
      /* Default Ratio */ new MotorRatio(1, 1),
      /* Forward Speed */ intakePower,
      /* Reverse Speed */ outtakePower * (3./4.),
      /* estimatedImpulseForceMetersPerSecond */ 1
    );
  }

  public static class ClimberConstants {

    public static int leftClimberId = 21;
    public static int rightClimberId = 22;
    public static final double maxHeight = 23; // Temp 
    public static final double minHeight = 0;

    public static final double roboWeight = 120; // Temp
    public static final boolean enableFF = true;
    public static final double torque = vortexStallTorque;
  }
  public static class ShooterConstants {

    public static int topId = 20;
    public static int botId = 19;

    public static MotorRatio shooterRatio = new MotorRatio(1, 0.8);
    public static MotorRatio simpleShooterRatio = new MotorRatio(.7, .7);

    public static double normalPower = 0.8;
    public static int anglerLeftId = 17;
    public static int anglerRightId = 18;
    public static AnglerConstants anglerConstants = new AnglerConstants(
      /* Min Angle */Rotation2d.fromDegrees(0), // temp
      /* Max Angle */Rotation2d.fromRotations(1.25), // temp
      /* Length Meters */Units.inchesToMeters(5), // temp
      /* Weight Kilos */Units.lbsToKilograms(7.625), // temp
      /* Parallel To Ground Angle */Rotation2d.fromDegrees(0), // temp
      /* PID */new MotionMagic(0.3, 0.0, 0.01, -1), // ff goes unused
      /* Stall Torque (Nm) */ vortexStallTorque,
      /* Enable FF */ false,
      /* Gear Ratio */ 306/10,
      0.2
    );

    public static ManipulatorConstants manipulatorConstants = new ManipulatorConstants(
      /* Is Inverted */ true,
      /* Default Ratio */ shooterRatio,
      /* Forward Speed */ normalPower,
      /* Reverse Speed */ normalPower / 4,
      /* estimatedImpulseForceMetersPerSecond */ 1
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
    public final double gearRatio;
    public final double speedLimit;

    public AnglerConstants(
      Rotation2d minAngle,
      Rotation2d maxAngle,
      double lengthMeters,
      double weight,
      Rotation2d parallelToGroundAngle,
      MotionMagic pid,
      double stallTorque,
      boolean enableFF,
      double gearRatio,
      double speedLimit
    ) {
      this.minAngle = minAngle;
      this.maxAngle = maxAngle;
      this.lengthMeters = lengthMeters;
      this.weight = weight;
      this.parallelToGroundAngle = parallelToGroundAngle;
      this.pid = pid;
      this.stalltorque = stallTorque;
      this.enableFF = enableFF;
      this.gearRatio = gearRatio;
      this.speedLimit = speedLimit;
    }
  }

  public static class ManipulatorConstants {
    public final boolean isInverted;
    public final Manipulator.MotorRatio defaultRatio;
    public final double forwardSpeed;
    public final double reverseSpeed;
    public final double estimatedImpulseForceMetersPerSecond;

    public ManipulatorConstants(boolean isInverted, Manipulator.MotorRatio defaultRatio, double forwardSpeed, double reverseSpeed, double estimatedImpulseForceMetersPerSecond) {
      this.isInverted = isInverted;
      this.defaultRatio = defaultRatio;
      this.forwardSpeed = forwardSpeed;
      this.reverseSpeed = reverseSpeed;
      this.estimatedImpulseForceMetersPerSecond = estimatedImpulseForceMetersPerSecond;
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
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5842;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5334;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Rotation2d.fromRotations(0).getRadians();
    //-Rotation2d.fromRotations(0.428223).plus(Rotation2d.fromDegrees(180)).getRadians();

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Rotation2d.fromRotations(0).getRadians();
    //-Rotation2d.fromRotations(0.413818).plus(Rotation2d.fromDegrees(180)).getRadians();

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Rotation2d.fromRotations(0).getRadians();
    //-Rotation2d.fromRotations(0.071533).plus(Rotation2d.fromDegrees(-180)).getRadians();

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Rotation2d.fromRotations(0).getRadians();
    //-Rotation2d.fromRotations(0.059326).plus(Rotation2d.fromDegrees(180)).getRadians();;

 public static final double MAX_VOLTAGE = 12.0;
 
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
    (6380.0 / 60.0) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
    MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public static final double MIN_ANGULAR_VELOCITY = 0.5;
  // Max input acceleration (ChassisSpeeds meters per second per second) for x/y
  // movement
  public static final double SLEW_RATE_LIMIT_TRANSLATION = MAX_VELOCITY_METERS_PER_SECOND * 2;
  // Max input acceleration (ChassisSpeeds radians per second per second) for
  // rotational movement
  public static final double SLEW_RATE_LIMIT_ROTATION = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 10;

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Front right
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

    

  }

  public static class VisionConstants {
    public static final Transform3d SHOOTER_CAMERA_POSE = new Transform3d(
      new Translation3d(Units.inchesToMeters(-13.5), Units.inchesToMeters(5),Units.inchesToMeters(8+1.87)),
      new Rotation3d(0, Units.degreesToRadians(-32.5), Units.degreesToRadians(180))
    ); //32.5
    public static final Resolution SHOOTER_CAMERA_RESOLUTION = Resolution.RES_640_480;
    public static final Rotation2d SHOOTER_CAMERA_FOV = Rotation2d.fromDegrees(76.2);

     public static final Transform3d INTAKE_CAMERA_POSE = new Transform3d(
      new Translation3d(Units.inchesToMeters(5), Units.inchesToMeters(8),Units.inchesToMeters(13+1.87)),
      new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))
      );


    public static final Transform3d DRIVER_CAMERA_POSE = new Transform3d(
      new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(0),Units.inchesToMeters(4)),
      new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))
    ); //32.5
    public static final Resolution DRIVER_CAMERA_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d DRIVER_CAMERA_FOV = Rotation2d.fromDegrees(70);

  }
  
  public static class TunerConstatns {
        // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(1.5).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);



    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = (6380.0 / 60.0) * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        kSpeedAt12VoltsMps / Math.hypot(DrivebaseConstant.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivebaseConstant.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 12.8;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = false;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "Omnivore2024";
    private static final int kPigeonId = 13;

    
    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 7;
    private static final int kFrontLeftSteerMotorId = 8;
    private static final int kFrontLeftEncoderId = 12;
    private static final double kFrontLeftEncoderOffset = -(0.258545);

    private static final double kFrontLeftXPosInches = 10.25;
    private static final double kFrontLeftYPosInches = 11.75;

    // Front Right
    private static final int kFrontRightDriveMotorId = 1;
    private static final int kFrontRightSteerMotorId = 2;
    private static final int kFrontRightEncoderId = 9;
    private static final double kFrontRightEncoderOffset = -(-0.019043);

    private static final double kFrontRightXPosInches = 10.25;
    private static final double kFrontRightYPosInches = -11.75;

    // Back Left
    private static final int kBackLeftDriveMotorId = 5;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 11;
    private static final double kBackLeftEncoderOffset = -(0.344727);

    private static final double kBackLeftXPosInches = -10.25;
    private static final double kBackLeftYPosInches = 11.75;

    // Back Right
    private static final int kBackRightDriveMotorId = 3;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 10;
    private static final double kBackRightEncoderOffset = -(-0.193359);

    private static final double kBackRightXPosInches = -10.25;
    private static final double kBackRightYPosInches = -11.75;

    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);




  }
}
