package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.team5431.titan.core.misc.Logger;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class DriveLockedRotCommand extends Command {

  private final Drivebase m_drivetrainSubsystem;

  private final double gyroAngle;

  private final BooleanSupplier isManualRotating;

  private PIDController rotController = new PIDController(6.0, 0.0, 0.0);

    private SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public DriveLockedRotCommand(Drivebase drivebase, double angle, BooleanSupplier shouldConclude) {
    this.m_drivetrainSubsystem = drivebase;
    this.gyroAngle = angle;
    this.isManualRotating = shouldConclude;
  }

  @Override
  public void initialize() {
    Logger.l("Going to rot " + gyroAngle);
    rotController = new PIDController(0.12, 0.12, 0.012);
    rotController.setTolerance(1, 2);
    rotController.enableContinuousInput(0, 360);
    rotController.setSetpoint(gyroAngle);
  }

  @Override
  public void execute() {
    double rot = MathUtil.clamp(rotController.calculate(m_drivetrainSubsystem.getPigeon2().getRotation2d().getDegrees()), -Constants.TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, Constants.TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented movement
    m_drivetrainSubsystem.applyRequest(() -> driveFC.withRotationalRate(rot));
  }

  @Override
  public void end(boolean interrupted) {
    Logger.l("Default drive ending");
  }

  @Override
  public boolean isFinished() {
    return rotController.atSetpoint() || isManualRotating.getAsBoolean();
  }
  
}
