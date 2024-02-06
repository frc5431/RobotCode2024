package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ApriltagConstants.zone;
import frc.robot.Systems;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Vision;
import frc.team5431.titan.core.misc.Logger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ApriltagGluing extends Command {

  private final Drivebase m_drivetrainSubsystem;
  private final Vision vision;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;
  private final zone desiredZone;


  private final BooleanSupplier isManualRotating;

  private PIDController rotController = new PIDController(6.0, 0.0, 0.0);

  public ApriltagGluing(
    Systems systems,
    Vision vision,
    zone desiredZone,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier m_rotationSupplier,
    BooleanSupplier isManualRotating
  ) {
    this.isManualRotating = isManualRotating;
    this.vision = vision;
    this.desiredZone = desiredZone;
    this.m_drivetrainSubsystem = systems.getDrivebase();
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = m_rotationSupplier;

    addRequirements(m_drivetrainSubsystem);
    setName("ApriltagGluing");
  }

  public ApriltagGluing(
    Systems systems,
    Vision vision,
    zone desiredZone,
    Supplier<Pair<Double, Double>> magThetaSupplier,
    DoubleSupplier m_rotationSupplier,
    BooleanSupplier isManualRotating
  ) {
    this(
      systems,
      vision,
      desiredZone,
      () -> magThetaSupplier.get().getFirst() * Math.cos(magThetaSupplier.get().getSecond()),
      () -> magThetaSupplier.get().getFirst() * Math.sin(magThetaSupplier.get().getSecond()),
      m_rotationSupplier,
      isManualRotating
    );
  }

  @Override
  public void initialize() {
    rotController = new PIDController(0.12, 0.01, 0.012);
    rotController.setTolerance(1, 2);
    rotController.enableContinuousInput(0, 360);
    double tagYaw = vision.getTargetYawZone(desiredZone, m_rotationSupplier.getAsDouble());
    double targYaw = (tagYaw == m_rotationSupplier.getAsDouble()) ? m_rotationSupplier.getAsDouble() : Math.atan2(tagYaw, m_rotationSupplier.getAsDouble());
    Logger.l("Going to rot " + targYaw);
    rotController.setSetpoint(targYaw);
  }

  @Override
  public void execute() {
    double x = m_translationXSupplier.getAsDouble();
    double y = m_translationYSupplier.getAsDouble();
    double rot = rotController.calculate(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees());

    rot = MathUtil.clamp(rot, -Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented movement
    m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rot, m_drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    Logger.l("Default drive ending");
    m_drivetrainSubsystem.stop();
  }


}
