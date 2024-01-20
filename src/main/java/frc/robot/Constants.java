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



  public static class Drivebase {
    
    public static final double maxSpeed = 4;

    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    
    public static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final double SlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SpeedAt12VoltsMps = 4.73;

    // Every 1 rotation of the azimuth results in CoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double CoupleRatio = 3.5714285714285716;

    public static final double DriveGearRatio = 6.746031746031747;
    public static final double SteerGearRatio = 12.8;
    public static final double WheelRadiusInches = 2;

    public static final boolean SteerMotorReversed = false;
    public static final boolean invertLeftSide = false;
    public static final boolean invertRightSide = true;

    public static final String CANbusName = "omnivore";
    public static final int PigeonId = 13;


    // These are only used for simulation
    public static final double SteerInertia = 0.00001;
    public static final double DriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    public static final double SteerFrictionVoltage = 0.25;
    public static final double DriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(PigeonId)
            .withCANbusName(CANbusName);

    public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(DriveGearRatio)
            .withSteerMotorGearRatio(SteerGearRatio)
            .withWheelRadius(WheelRadiusInches)
            .withSlipCurrent(SlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(SpeedAt12VoltsMps)
            .withSteerInertia(SteerInertia)
            .withDriveInertia(DriveInertia)
            .withSteerFrictionVoltage(SteerFrictionVoltage)
            .withDriveFrictionVoltage(DriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(CoupleRatio)
            .withSteerMotorInverted(SteerMotorReversed);


    // front Left
    public static final int frontLeftDriveMotorId = 4;
    public static final int frontLeftSteerMotorId = 3;
    public static final int frontLeftEncoderId = 10;
    public static final double frontLeftEncoderOffset = -0.245849609375;

    public static final double frontLeftXPosInches = 12.75;
    public static final double frontLeftYPosInches = 10.75;

    // front Right
    public static final int frontRightDriveMotorId = 8;
    public static final int frontRightSteerMotorId = 7;
    public static final int frontRightEncoderId = 12;
    public static final double frontRightEncoderOffset = -0.291015625;

    public static final double frontRightXPosInches = 12.75;
    public static final double frontRightYPosInches = -10.75;

    // back Left
    public static final int backLeftDriveMotorId = 6;
    public static final int backLeftSteerMotorId = 5;
    public static final int backLeftEncoderId = 11;
    public static final double backLeftEncoderOffset = -0.18994140625;

    public static final double backLeftXPosInches = -12.75;
    public static final double backLeftYPosInches = 10.75;

    // Back Right
    public static final int backRightDriveMotorId = 2;
    public static final int backRightSteerMotorId = 1;
    public static final int backRightEncoderId = 9;
    public static final double backRightEncoderOffset = -0.313232421875;

    public static final double backRightXPosInches = -12.75;
    public static final double backRightYPosInches = -10.75;


    public static final SwerveModuleConstants frontLeft = ConstantCreator.createModuleConstants(
            frontLeftSteerMotorId, frontLeftDriveMotorId, frontLeftEncoderId, frontLeftEncoderOffset, 
            Units.inchesToMeters(frontLeftXPosInches), Units.inchesToMeters(frontLeftYPosInches), invertLeftSide);

    public static final SwerveModuleConstants frontRight = ConstantCreator.createModuleConstants(
            frontRightSteerMotorId, frontRightDriveMotorId, frontRightEncoderId, frontRightEncoderOffset, 
            Units.inchesToMeters(frontRightXPosInches), Units.inchesToMeters(frontRightYPosInches), invertRightSide);

    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            backLeftSteerMotorId, backLeftDriveMotorId, backLeftEncoderId, backLeftEncoderOffset, 
            Units.inchesToMeters(backLeftXPosInches), Units.inchesToMeters(backLeftYPosInches), invertLeftSide);

    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            backRightSteerMotorId, backRightDriveMotorId, backRightEncoderId, backRightEncoderOffset, 
            Units.inchesToMeters(backRightXPosInches), Units.inchesToMeters(backRightYPosInches), invertRightSide);

  }



}


