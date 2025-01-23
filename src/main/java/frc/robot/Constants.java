package frc.robot;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.MotorRatio;
import frc.team5431.titan.core.robot.MotionMagic;

public final class Constants {
  public static final boolean isGals = false;
  public static double vortexStallTorque = 3.6;
  public static double neoStallTorque = 2.6;
  public static double neo550StallTorque = 0.97;

  public static class idsorsmth {

    public static final int idorsmth = 0;
    public static final boolean isTauseefCool = true;
  }

  public static class ApriltagConstants {

    public enum AprilTagZone {
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
    public static double rotationTolerance = 0.003;
    public static Angle mainAngle = edu.wpi.first.units.Units.Degree.of(173);
    public static Angle distantStowAngle = edu.wpi.first.units.Units.Degree.of(200);
    //55
    public enum IntakeModes {
      INTAKE,
      OUTAKE,
      STOPPED
    }

    public static AnglerConstants anglerConstants = new AnglerConstants(
        /* Min Angle */edu.wpi.first.units.Units.Degree.of(315),
        /* Main Angle */edu.wpi.first.units.Units.Degree.of(150),
        /* Length Kilos */Units.Meters.convertFrom(12, Units.Inches),
        /* Length Meters */Units.Kilograms.convertFrom(8, Units.Pounds),
        /* Parallel To Ground Angle */edu.wpi.first.units.Units.Rotation.of(0.824),
        /* PID */new MotionMagic(0.8, 0.0, 0.45, -1),
        /* Stall Torque (Nm) */ neoStallTorque,
        /* Enable FF */ true,
        /* Gear Ratio */20.625,
        0.8,
        true);

    public static ManipulatorConstants manipulatorConstants = new ManipulatorConstants(
        /* Is Inverted */ true,
        /* Default Ratio */ new MotorRatio(1, 1),
        /* Forward Speed */ intakePower,
        /* Reverse Speed */ outtakePower * (3. / 4.),
        /* estimatedImpulseForceMetersPerSecond */ 1);
  }

  public static class AmperConstants {

    public static double intakePower = -0.5;
    public static double outtakePower = 0.3;
    public static int amperId = 23;
    public static int amperPivotId = 24;
    public static double radianTolerance = 0.03;

    public enum AmperModes {
      INTAKE,
      OUTAKE,
      STOPPED
    }

     public static AnglerConstants anglerConstants = new AnglerConstants(
        /* Min Angle */edu.wpi.first.units.Units.Degree.of(5),
        /* Main Angle */edu.wpi.first.units.Units.Degree.of(171),
        /* Length Kilos */Units.Meters.convertFrom(8, Units.Inches),
        /* Length Meters */Units.Kilograms.convertFrom(4, Units.Pounds),
        /* Parallel To Ground Angle */edu.wpi.first.units.Units.Rotation.of(0.174),
        /* PID */new MotionMagic(0.5, 0.04, 0.3, -1),
        /* Stall Torque (Nm) */ neoStallTorque,
        /* Enable FF */ false,
        /* Gear Ratio */9,
        0.5,
        false);

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

    public static int mainTopId = 20;
    public static int mainBotId = 19;
    public static int distantTopId = 18;
    public static int distantBotId = 17;

    public static double spkSpeed = -1.;
    public static double spkDist = -0.91;
    public static double dangerDist = -.65;
    
    public static double ampSpeed = -0.4;
    public static double stgSpeed = -1.;
    public static double inSpeed = 0.2;

    public static double mainAngle = 55;
    public static double secondaryAngle = 35;// ?

    //public static Pair<Double, Double> spkRatio = Pair.of(0.9, 1.);
    public static Pair<Double, Double> ampRatio = Pair.of(0.17, 0.51);
    public static Pair<Double, Double> mainRatio = Pair.of(0.9, 0.5);
    public static Pair<Double, Double> distRat = Pair.of(.67, .88);
    public static Pair<Double, Double> distDangerRat = Pair.of(1., 0.8);
    public static Pair<Double, Double> stageRat = Pair.of(0.86, 0.95);

    public static final double mainTopRpm = -5000;
    public static final double mainStageRpm = -4700;
    public static final double mainAmpRpm = -300;
    public static final double distTopRpm = 3600;
    public static final double distDangerRpm = 3800;

    public static final double p = 3;
    public static final double i = 0.0;
    public static final double d = 0.01;

    public enum ShooterModes {
      SpeakerShot(ShooterConstants.spkSpeed, mainRatio),
      AmpShot(ShooterConstants.ampSpeed, ampRatio), // has ratio
      StageShot(ShooterConstants.stgSpeed, stageRat), // has ratio
      SpeakerDistant(ShooterConstants.spkDist, distRat, false, true),
      DangerDistant(ShooterConstants.dangerDist, distDangerRat, false, true),
      REVERSE(ShooterConstants.inSpeed, true, true),
      NONE(0);

      public double speed;
      public Optional<Pair<Double, Double>> ratio;
      public boolean usesMain;
      public boolean usesDistant;

      ShooterModes(double speed) {
        this(speed, true, false);
      }

      ShooterModes(double speed, boolean usesMain, boolean usesDistant) {
        this.speed = speed;
        this.ratio = Optional.empty();
        this.usesMain = usesMain;
        this.usesDistant = usesDistant;
      }

      ShooterModes(double speed, Pair<Double, Double> ratio) {
        this(speed, ratio, true, false);
      }

      ShooterModes(double speed, Pair<Double, Double> ratio, boolean usesMain, boolean usesDistant) {
        this.speed = speed;
        this.ratio = Optional.of(ratio);
        this.usesMain = usesMain;
        this.usesDistant = usesDistant;
      }

    }

    public static MotorRatio shooterRatio = new MotorRatio(1, 0.95);
    public static MotorRatio simpleShooterRatio = new MotorRatio(.7, .7);

  }

  public static class AnglerConstants {

    public final Angle minAngle;
    public final Angle mainAngle;
    public final double lengthMeters;
    public final double weight;
    public final Angle parallelToGroundAngle;
    public final MotionMagic pid;
    public final double stalltorque;
    public final boolean enableFF;
    public final double gearRatio;
    public final double speedLimit;
    public final boolean enableWrapping;

    public AnglerConstants(
        Angle minAngle,
        Angle mainAngle,
        double lengthMeters,
        double weight,
        Angle parallelToGroundAngle,
        MotionMagic pid,

        double stallTorque,
        boolean enableFF,
        double gearRatio,
        double speedLimit,
        boolean enableWrapping) {
      this.minAngle = minAngle;
      this.mainAngle = mainAngle;
      this.lengthMeters = lengthMeters;
      this.weight = weight;
      this.parallelToGroundAngle = parallelToGroundAngle;
      this.pid = pid;
      this.stalltorque = stallTorque;
      this.enableFF = enableFF;
      this.gearRatio = gearRatio;
      this.speedLimit = speedLimit;
      this.enableWrapping = enableWrapping;
    }
  }

  public static class ManipulatorConstants {
    public final boolean isInverted;
    public final Intake.MotorRatio defaultRatio;
    public final double intakeSpeed;
    public final double outakeSpeed;
    public final double estimatedImpulseForceMetersPerSecond;

    public ManipulatorConstants(boolean isInverted, Intake.MotorRatio defaultRatio, double forwardSpeed,
        double reverseSpeed, double estimatedImpulseForceMetersPerSecond) {
      this.isInverted = isInverted;
      this.defaultRatio = defaultRatio;
      this.intakeSpeed = forwardSpeed;
      this.outakeSpeed = reverseSpeed;
      this.estimatedImpulseForceMetersPerSecond = estimatedImpulseForceMetersPerSecond;
    }
  }

  public static class VisionConstants {}
    // public static final Resolution ARDUCAM_CAMERA_RESOLUTION = Resolution.RES_640_480;

    // public static final Transform3d SHOOTER_CAMERA_POSE = new Transform3d(
    //     new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-6), Units.inchesToMeters(24)),
    //     new Rotation3d(0, Units.degreesToRadians(-32.5), Units.degreesToRadians(180))); // 32.5
    // public static final Rotation2d SHOOTER_CAMERA_FOV = Rotation2d.fromDegrees(76.2);

    // public static final Transform3d INTAKE_CAMERA_POSE = new Transform3d(
    //     new Translation3d(Units.inchesToMeters(-5), Units.inchesToMeters(-8), Units.inchesToMeters(13 + 1.87)),
    //     new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0)));

    // public static final Transform3d DRIVER_CAMERA_POSE = new Transform3d(
    //     new Translation3d(Units.inchesToMeters(13.5), Units.inchesToMeters(0), Units.inchesToMeters(4)),
    //     new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(0))); // 32.5
    // public static final Resolution DRIVER_CAMERA_RESOLUTION = Resolution.RES_1280_720;
    // public static final Rotation2d DRIVER_CAMERA_FOV = Rotation2d.fromDegrees(70);

//   }
// public static class TunerConstatns {

  
//     // Both sets of gains need to be tuned to your individual robot.

//     // The steer motor uses any SwerveModule.SteerRequestType control request with the
//     // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
//     private static final Slot0Configs steerGains = new Slot0Configs()
//         .withKP(0.1).withKI(0).withKD(0.5)
//         .withKS(0.1).withKV(1.91).withKA(0)
//         .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
//     // When using closed-loop control, the drive motor uses the control
//     // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
//     private static final Slot0Configs driveGains = new Slot0Configs()
//         .withKP(0.1).withKI(0).withKD(0)
//         .withKS(0).withKV(0.124);

//     // The closed-loop output type to use for the steer motors;
//     // This affects the PID/FF gains for the steer motors
//     private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
//     // The closed-loop output type to use for the drive motors;
//     // This affects the PID/FF gains for the drive motors
//     private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

//     // The type of motor used for the drive motor
//     private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
//     // The type of motor used for the drive motor
//     private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

//     // The remote sensor feedback type to use for the steer motors;
//     // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
//     private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

//     // The stator current at which the wheels start to slip;
//     // This needs to be tuned to your individual robot
//     private static final Current kSlipCurrent = Amps.of(120.0);

//     // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
//     // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
//     private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
//     private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
//         .withCurrentLimits(
//             new CurrentLimitsConfigs()
//                 // Swerve azimuth does not require much torque output, so we can set a relatively low
//                 // stator current limit to help avoid brownouts without impacting performance.
//                 .withStatorCurrentLimit(Amps.of(60))
//                 .withStatorCurrentLimitEnable(true)
//         );
//     // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
//     private static final Pigeon2Configuration pigeonConfigs = null;

//     // CAN bus that the devices are located on;
//     // All swerve devices must share the same CAN bus
//     public static final CANBus kCANBus = new CANBus("Omnivore2024", "./logs/example.hoot");

   
//     public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5842;

//     public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5334;


//     // Theoretical free speed (m/s) at 12 V applied output;
//     // This needs to be tuned to your individual robot
//     public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(0);

//     public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =0;

//     //public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = kSpeedAt12VoltsMps / Math.hypot(
//     //    DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

//     // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
//     // This may need to be tuned to your individual robot
//     private static final double kCoupleRatio = 3.5714285714285714285714285714286;

//     private static final double kDriveGearRatio = 6.746031746031747;
//     private static final double kSteerGearRatio = 12.8;
//     private static final double kWheelRadiusInches = 2;

//     private static final boolean kSteerMotorReversed = false;
//     private static final boolean kInvertLeftSide = false;
//     private static final boolean kInvertRightSide = false;
//     private static final double kDriveFrictionVoltage = 0.25;

//     public static final ModuleConfig moduleConfig = new ModuleConfig(1.0, kSpeedAt12Volts.baseUnitMagnitude(), kDriveFrictionVoltage, DCMotor.getFalcon500(4), 80, 0);
//     public static final RobotConfig robotConfig = new RobotConfig(125.0, 0.001, moduleConfig, DRIVETRAIN_TRACKWIDTH_METERS);
  

//     private static final String kCANbusName = "Omnivore2024";
//     private static final int kPigeonId = 13;

//     // These are only used for simulation
//     private static final double kSteerInertia = 0.00001;
//     private static final double kDriveInertia = 0.001;
//     // Simulated voltage necessary to overcome friction
//     private static final double kSteerFrictionVoltage = 0.25;

//     public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
//             .withCANBusName(kCANBus.getName())
//             .withPigeon2Id(kPigeonId)
//             .withPigeon2Configs(pigeonConfigs);
    
//     public static final MagnetSensorConfigs magConfig = new MagnetSensorConfigs()
//       .withAbsoluteSensorDiscontinuityPoint(1);
//     ;

//     public static final CANcoderConfiguration canConfig = new CANcoderConfiguration()
//       .withMagnetSensor(magConfig)
//     ;
    

//     private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
//         new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
//             .withDriveMotorGearRatio(kDriveGearRatio)
//             .withSteerMotorGearRatio(kSteerGearRatio)
//             .withCouplingGearRatio(kCoupleRatio)
//             .withWheelRadius(kWheelRadiusInches)
//             .withSteerMotorGains(steerGains)
//             .withDriveMotorGains(driveGains)
//             .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
//             .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
//             .withSlipCurrent(kSlipCurrent)
//             .withSpeedAt12Volts(kSpeedAt12Volts)
//             .withDriveMotorType(kDriveMotorType)
//             .withSteerMotorType(kSteerMotorType)
//             .withFeedbackSource(kSteerFeedbackType)
//             .withDriveMotorInitialConfigs(driveInitialConfigs)
//             .withSteerMotorInitialConfigs(steerInitialConfigs)
//             .withEncoderInitialConfigs(canConfig)
//             .withSteerInertia(kSteerInertia)
//             .withDriveInertia(kDriveInertia)
//             .withSteerFrictionVoltage(kSteerFrictionVoltage)
//             .withDriveFrictionVoltage(kDriveFrictionVoltage);


//     // Front Left
//     private static final int kFrontLeftDriveMotorId = 7;
//     private static final int kFrontLeftSteerMotorId = 8;
//     private static final int kFrontLeftEncoderId = 12;
//     private static final Angle kFrontLeftEncoderOffset = Rotation.of(-0.232422+0.5);
//     private static final boolean kFrontLeftSteerMotorInverted = true;
//     private static final boolean kFrontLeftEncoderInverted = false;

//     private static final Distance kFrontLeftXPos = Inches.of(10);
//     private static final Distance kFrontLeftYPos = Inches.of(10);

//     // Front Right
//     private static final int kFrontRightDriveMotorId = 1;
//     private static final int kFrontRightSteerMotorId = 2;
//     private static final int kFrontRightEncoderId = 9;
//     private static final Angle kFrontRightEncoderOffset = Rotation.of(-0.024658+0.5);
//     private static final boolean kFrontRightSteerMotorInverted = true;
//     private static final boolean kFrontRightEncoderInverted = false;

//     private static final Distance kFrontRightXPos = Inches.of(10);
//     private static final Distance kFrontRightYPos = Inches.of(-10);

//     // Back Left
//     private static final int kBackLeftDriveMotorId = 5;
//     private static final int kBackLeftSteerMotorId = 6;
//     private static final int kBackLeftEncoderId = 11;
//     private static final Angle kBackLeftEncoderOffset = Rotation.of(-0.161133+0.5);
//     private static final boolean kBackLeftSteerMotorInverted = true;
//     private static final boolean kBackLeftEncoderInverted = false;

//     private static final Distance kBackLeftXPos = Inches.of(-10);
//     private static final Distance kBackLeftYPos = Inches.of(10);

//     // Back Right
//     private static final int kBackRightDriveMotorId = 3;
//     private static final int kBackRightSteerMotorId = 4;
//     private static final int kBackRightEncoderId = 10;
//     private static final Angle kBackRightEncoderOffset = Rotation.of(-0.194824+0.5);
//     private static final boolean kBackRightSteerMotorInverted = true;
//     private static final boolean kBackRightEncoderInverted = false;

//     private static final Distance kBackRightXPos = Inches.of(-10);
//     private static final Distance kBackRightYPos = Inches.of(-10);


//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
//         ConstantCreator.createModuleConstants(
//             kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
//             kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
//         );
//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
//         ConstantCreator.createModuleConstants(
//             kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
//             kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
//         );
//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
//         ConstantCreator.createModuleConstants(
//             kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
//             kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
//         );
//     public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
//         ConstantCreator.createModuleConstants(
//             kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
//             kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
//         );

//     /**
//      * Creates a Drivebase instance.
//      * This should only be called once in your robot program,.
//      */
//     // public static Drivebase createDrivetrain() {
//     //     return new Drivebase(
//     //         DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
//     //     );
//     // }


//     /**
//      * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
//      */
//     public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
//         /**
//          * Constructs a CTRE SwerveDrivetrain using the specified constants.
//          * <p>
//          * This constructs the underlying hardware devices, so users should not construct
//          * the devices themselves. If they need the devices, they can access them through
//          * getters in the classes.
//          *
//          * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
//          * @param modules               Constants for each specific module
//          */
//         public TunerSwerveDrivetrain(
//             SwerveDrivetrainConstants drivetrainConstants,
//             SwerveModuleConstants<?, ?, ?>... modules
//         ) {
//             super(
//                 TalonFX::new, TalonFX::new, CANcoder::new,
//                 drivetrainConstants, modules
//             );
//         }

//         /**
//          * Constructs a CTRE SwerveDrivetrain using the specified constants.
//          * <p>
//          * This constructs the underlying hardware devices, so users should not construct
//          * the devices themselves. If they need the devices, they can access them through
//          * getters in the classes.
//          *
//          * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
//          * @param odometryUpdateFrequency The frequency to run the odometry loop. If
//          *                                unspecified or set to 0 Hz, this is 250 Hz on
//          *                                CAN FD, and 100 Hz on CAN 2.0.
//          * @param modules                 Constants for each specific module
//          */
//         public TunerSwerveDrivetrain(
//             SwerveDrivetrainConstants drivetrainConstants,
//             double odometryUpdateFrequency,
//             SwerveModuleConstants<?, ?, ?>... modules
//         ) {
//             super(
//                 TalonFX::new, TalonFX::new, CANcoder::new,
//                 drivetrainConstants, odometryUpdateFrequency, modules
//             );
//         }

//         /**
//          * Constructs a CTRE SwerveDrivetrain using the specified constants.
//          * <p>
//          * This constructs the underlying hardware devices, so users should not construct
//          * the devices themselves. If they need the devices, they can access them through
//          * getters in the classes.
//          *
//          * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
//          * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
//          *                                  unspecified or set to 0 Hz, this is 250 Hz on
//          *                                  CAN FD, and 100 Hz on CAN 2.0.
//          * @param odometryStandardDeviation The standard deviation for odometry calculation
//          *                                  in the form [x, y, theta]ᵀ, with units in meters
//          *                                  and radians
//          * @param visionStandardDeviation   The standard deviation for vision calculation
//          *                                  in the form [x, y, theta]ᵀ, with units in meters
//          *                                  and radians
//          * @param modules                   Constants for each specific module
//          */
//         public TunerSwerveDrivetrain(
//             SwerveDrivetrainConstants drivetrainConstants,
//             double odometryUpdateFrequency,
//             Matrix<N3, N1> odometryStandardDeviation,
//             Matrix<N3, N1> visionStandardDeviation,
//             SwerveModuleConstants<?, ?, ?>... modules
//         ) {
//             super(
//                 TalonFX::new, TalonFX::new, CANcoder::new,
//                 drivetrainConstants, odometryUpdateFrequency,
//                 odometryStandardDeviation, visionStandardDeviation, modules
//             );
//         }
//     }
//   }
}