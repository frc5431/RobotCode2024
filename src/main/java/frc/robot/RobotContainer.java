// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.Constants.TunerConstatns;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.auton.AmpScore;
import frc.robot.commands.auton.AutoIntakeNote;
import frc.robot.commands.auton.DistantSpeakerScore;
import frc.robot.commands.auton.IntakeNote;
import frc.robot.commands.auton.SimpleSpeaker;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.swerve.TitanFieldCentricFacingAngle;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import frc.team5431.titan.core.misc.Logger;

public class RobotContainer {

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Systems systems = new Systems();
  private final Drivebase drivebase = systems.getDrivebase();

  private final Pivot pivot = systems.getPivot();
  // private final Climber climber = systems.getClimber();

  private final Intake intake = systems.getIntake();
  private final Shooter shooter = systems.getShooter();
  private final AutonMagic autonMagic;
  private double vibrateTime = 0;
  private boolean shouldBeVibrating = false;
  private boolean oldBeamBreakStatus = true;
  double unchangedBeamBreakTime = 0;
  public Rotation2d targetRotation;
  TitanFieldCentricFacingAngle facingRequest = new TitanFieldCentricFacingAngle();

  IntakeNote intakeNote = new IntakeNote(intake, pivot);
  SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  private SwerveRequest.RobotCentric driveRo = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {
    NamedCommands.registerCommand("AmpScore", new AmpScore(shooter, intake));
    NamedCommands.registerCommand("SpeakerScore", new SimpleSpeaker(shooter, intake));
    NamedCommands.registerCommand("DistantSpeakerScore", new DistantSpeakerScore(shooter, intake));
    NamedCommands.registerCommand("GrabNote", new AutoIntakeNote(intake, pivot));

    autonMagic = new AutonMagic();

    // drivebase.seedFieldRelative();
    facingRequest.withPID(new PIDController(6, 0.01, 0.008));
    facingRequest.withDampening(new GeneralArtificalIntelModelFizzBuzzEnterpriseRLMachineLearnedMoneyNFTCryptoCoinZooEggController(35));
    facingRequest.gyro = drivebase.getGyro();
    configureBindings();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    // var alliance = DriverStation.getAlliance();
    // if(alliance.get() == DriverStation.Alliance.Red) {
    // value = -value;
    // }

    value = deadband(value, 0.15);

    // More sensitive at smaller speeds
    double newValue = Math.pow(value, 2);

    // Copy the sign to the new value
    newValue = Math.copySign(newValue, value);

    return newValue;
  }

  public void periodic() {
    var angle = systems.getSimpleVision().getAngleToSpeaker();
    if(angle.isPresent()) {
      SmartDashboard.putNumber("Target", angle.get().getRadians());
    }
    

    if (shooter.mode == ShooterModes.NONE) {
      pivot.setpoint = pivot.setpoint;
    } else if (shooter.mode.usesDistant) {
      var encoderPos = edu.wpi.first.math.util.Units.radiansToDegrees(pivot.absoluteEncoder.getPosition());
      if (encoderPos >= 200) {
        pivot.setpoint = Constants.IntakeConstants.ampAngle;
        return;
      }
      pivot.setpoint = IntakeConstants.distantStowAngle;
      intakeNote.cancel();
    } else if (shooter.mode.usesMain) {
      pivot.setpoint = IntakeConstants.anglerConstants.mainAngle;
      intakeNote.cancel();
    }

    boolean beamBreakStatus = intake.getBeamBreakStatus().get();
    unchangedBeamBreakTime += 0.02;
    if (beamBreakStatus != oldBeamBreakStatus) {
      unchangedBeamBreakTime = 0;
    }

    if (!beamBreakStatus && unchangedBeamBreakTime == 0.1) {
      shouldBeVibrating = true;
      vibrateTime = 1;
    }
    if (shouldBeVibrating) {
      driver.getHID().setRumble(RumbleType.kLeftRumble, 0.25);
      vibrateTime -= 0.02;
      if (vibrateTime <= 0) {
        shouldBeVibrating = false;
      }
    } else {
      driver.getHID().setRumble(RumbleType.kLeftRumble, 0);
    }

    oldBeamBreakStatus = beamBreakStatus;

    operator.getHID().setRumble(RumbleType.kLeftRumble, (shooter.isClose(200)) ? 0.5 : 0);
  }

  private void configureBindings() {

    drivebase.setDefaultCommand( // Drivetrain will execute this command periodically
        drivebase.applyRequest(() -> {
          double u = driver.getLeftX();
          double v = driver.getLeftY();

          double root2 = Math.sqrt(2);
          double magnitude = Math.sqrt(u * u + v * v);
          double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
          double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);

          return driveFC
              .withVelocityX(
                  modifyAxis(y2 + (driver.povUp().getAsBoolean() ? 0.1 : 0))
                      * TunerConstatns.kSpeedAt12VoltsMps)
              .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps)
              .withRotationalRate(
                  modifyAxis(driver.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        }));

    driver.a().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(drivebase.applyRequest(() -> {
      double u = driver.getLeftX();
      double v = driver.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
      return facingRequest
          .withVelocityX(
              modifyAxis(y2 + (driver.povUp().getAsBoolean() ? 0.1 : 0))
                  * TunerConstatns.kSpeedAt12VoltsMps)
          .withHeading(edu.wpi.first.math.util.Units.degreesToRadians(38))
          .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps);
    }).until(() -> Math.abs(driver.getRawAxis(4)) > 0.15));

    driver.b().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).whileTrue(drivebase.applyRequest(() -> {
      double u = driver.getLeftX();
      double v = driver.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
      var angle = systems.getSimpleVision().getAngleToSpeaker();
      if (angle.isPresent()) {
        return facingRequest
            .withHeading(angle.get().plus(Rotation2d.fromDegrees(drivebase.getGyro().getAngle())).getRadians())
            .withVelocityX(
                modifyAxis(y2 + (driver.povUp().getAsBoolean() ? 0.1 : 0))
                    * TunerConstatns.kSpeedAt12VoltsMps)
            .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps);
      }

      return facingRequest
          .withVelocityX(
              modifyAxis(y2 + (driver.povUp().getAsBoolean() ? 0.1 : 0))
                  * TunerConstatns.kSpeedAt12VoltsMps)
          .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps);
    }));

    driver.rightBumper().whileTrue(drivebase.applyRequest(() -> {
      double u = driver.getLeftX();
      double v = driver.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
      return driveRo
              .withVelocityX(
                  modifyAxis(y2 + (driver.povUp().getAsBoolean() ? 0.1 : 0))
                      * TunerConstatns.kSpeedAt12VoltsMps)
              .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps)
              .withRotationalRate(
                  modifyAxis(driver.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }));

    

    driver.y().onTrue(new InstantCommand(() -> drivebase.resetGyro()));
    driver.leftTrigger()
        .onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, pivot).andThen(() -> intakeNote.cancel()));
    // driver.leftBumper().onTrue(drivebase.);
    // driver.a().onTrue(new RunClimberCommand(climber,
    // RunClimberCommand.ClimberMode.EXTENDED));
    // driver.rightTrigger().whileTrue(climber.increment(driver.getRightTriggerAxis()
    // * 0.1));
    // driver.leftBumper().onTrue(new RunClimberCommand(climber,
    // RunClimberCommand.ClimberMode.RETRACTED));

    // Shooter
    operator.rightTrigger().whileTrue(shooter.runShooterCommand(ShooterModes.SpeakerShot));
    operator.b().whileTrue(shooter.runShooterCommand(ShooterModes.REVERSE));
    operator.a().whileTrue(shooter.runShooterCommand(ShooterModes.SpeakerDistant));
    operator.y().whileTrue(shooter.runShooterCommand(ShooterModes.StageShot));
    operator.start().whileTrue(shooter.runShooterCommand(ShooterModes.AmpShot));
    operator.povUp().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot));
    operator.povDown().whileTrue(shooter.runShooterCommand(ShooterModes.DangerDistant));

    // Intake
    operator.leftTrigger().whileTrue(RunManipulatorCommand.withMode(intake, IntakeModes.INTAKE));
    operator.x().whileTrue(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE));

    // Intake Angler
    operator.axisGreaterThan(1, 0.15)
        .whileTrue(new RunCommand(() -> pivot.increment(-operator.getLeftY() * 1), pivot).repeatedly());
    operator.axisLessThan(1, -0.15)
        .whileTrue(new RunCommand(() -> pivot.increment(-operator.getLeftY() * 1), pivot).repeatedly());

    operator.back()
        .onTrue(new InstantCommand(() -> pivot.setRotation(Constants.IntakeConstants.ampAngle), pivot));
    operator.leftBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, pivot));
    operator.rightBumper().onTrue(intakeNote);

  }

  public Command getAutonomousCommand() {
    return autonMagic.procureAuton();
  }

  public void onTeleop() {
    pivot.setpoint = Units.Radians.of(pivot.absoluteEncoder.getPosition());
  }

}