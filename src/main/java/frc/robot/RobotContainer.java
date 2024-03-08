// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TunerConstatns;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.auton.AmpScore;
import frc.robot.commands.auton.IntakeNote;

import frc.robot.controllers.DriverController;
import frc.robot.controllers.DriverSkyflyController;
import frc.robot.controllers.DriverXboxController;
import frc.robot.subsystems.Angler;
//import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PheonixDrivebase;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {
  private final DriverController driver;
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Systems systems = new Systems();
  private final PheonixDrivebase drivebase = systems.getDrivebase();
  private final Angler pivot = systems.getPivot();
  private final Manipulator intake = systems.getIntake();
  private final Manipulator shooter = systems.getShooter();
  private final AutonMagic autonMagic;

  private SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // private SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // private boolean isFieldRelative;

  // public void setFieldRelativeControl() {
  //   isFieldRelative = true;
  // }

  // public void setRobotRelativeControl() {
  //   isFieldRelative = false;
  // }

  public RobotContainer() {
    NamedCommands.registerCommand("AmpScore", new AmpScore(intake, pivot));
    NamedCommands.registerCommand("GrabNote", new IntakeNote(intake, pivot));
    NamedCommands.registerCommand("SpeakerScore", new WaitCommand(2));


    autonMagic = new AutonMagic(systems);


    if(Constants.useXboxController) {
      driver = new DriverXboxController();
    }else {
      driver = new DriverSkyflyController();
    }

    drivebase.seedFieldRelative();
    // setFieldRelativeControl();
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
    value = deadband(value, 0.15);

    // More sensitive at smaller speeds
    double newValue = Math.pow(value, 2);

    // Copy the sign to the new value
    newValue = Math.copySign(newValue, value);

    return newValue;
  }

  public void periodic() {
    //SmartDashboard.putNumberArray("Shooter Speeds", shooter.getRPM());
    //SmartDashboard.putNumberArray("Intake Speeds", intake.getRPM());

    //SmartDashboard.putNumber("dx", driver.getLeftX());
    //SmartDashboard.putNumber("dy", driver.getLeftY());
  }

  Translation2d ellipticalDiscToSquare(double u, double v) {
    double u2 = u * u;
    double v2 = v * v;
    double twosqrt2 = 2.0 * Math.sqrt(2.0);
    double subtermx = 2.0 + u2 - v2;
    double subtermy = 2.0 - u2 + v2;
    double termx1 = subtermx + u * twosqrt2;
    double termx2 = subtermx - u * twosqrt2;
    double termy1 = subtermy + v * twosqrt2;
    double termy2 = subtermy - v * twosqrt2;

    double x = MathUtil.clamp(0.5 * Math.sqrt(termx1) - 0.5 * Math.sqrt(termx2), -1, 1);
    double y = MathUtil.clamp(0.5 * Math.sqrt(termy1) - 0.5 * Math.sqrt(termy2), -1, 1);

    if(Math.abs(x) > 0.9 && Math.abs(y) < 0.2) {
      x = Math.copySign(1, x);
    }

    if(Math.abs(y) > 0.9 && Math.abs(x) < 0.2) {
      y = Math.copySign(1, y);
    }
    return new Translation2d(x, y) ;
  }

  Translation2d FGSquircularMap(double u, double v) {
    double sgnuv = Math.signum(u * v);
    double sqrt2 = Math.sqrt(2);
    double root = Math.sqrt((u*u) + (v*v) - Math.sqrt(((u*u) + (v*v)) * ((u*u) + (v*v) - 4 * (u*u) * (v*v))));

    double x = (sgnuv / (v * sqrt2)) * ( root);
    double y = (sgnuv / (u * sqrt2)) * ( root);

    if(Math.abs(x) > 0.85 && Math.abs(y) < 0.2) {
      x = Math.copySign(1, x);
    }

    if(Math.abs(y) > 0.85 && Math.abs(x) < 0.2) {
      y = Math.copySign(1, y);
    }

    return new Translation2d(
      x,
      y
    );
  }

  private void configureBindings() {
    shooter.setRatio(Constants.ShooterConstants.simpleShooterRatio);

    // driver.setDeadzone(0.15);
    /*
     * // drivebase.setDefaultCommand(
     * // new DefaultDriveCommand(
     * // systems,
     * // (Supplier<Pair<Double, Double>>) () -> {
     * // double inX = -driver.getLeftY(); // swap intended
     * // if(driver.povUp().getAsBoolean()) {
     * // inX = 0.3;
     * // }
     * // double inY = -driver.getLeftX();
     * // double mag = Math.hypot(inX, inY);
     * // double theta = Math.atan2(inY, inX);
     * // return Pair.of(modifyAxis(mag) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
     * theta);
     * // },
     * // () -> -modifyAxis(-driver.getRightX()) *
     * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
     */
    drivebase.setDefaultCommand( // Drivetrain will execute this command periodically
        drivebase.applyRequest(() -> {
          // var axis = FGSquircularMap(driver.getLeftX() * .89, driver.getLeftY() * 0.89);
          return driveFC.withVelocityX(modifyAxis(driver.getLeftY()) * TunerConstatns.kSpeedAt12VoltsMps)
            .withVelocityY(modifyAxis(driver.getLeftX()) * TunerConstatns.kSpeedAt12VoltsMps)
            .withRotationalRate(
                -modifyAxis(driver.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        }));

    driver.resetGyro().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
    if(driver instanceof DriverSkyflyController) {
      driver.resetGyro().onFalse(new InstantCommand(() -> drivebase.zeroGyro()));
    }

    // SmartDashboard.putNumber("turn axis",
    //     -modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    // Shooter
    operator.rightTrigger().whileTrue(RunManipulatorCommand.withPower(shooter, -1));
    operator.b().whileTrue(RunManipulatorCommand.withPower(shooter, 1));

    // Intake
    operator.leftTrigger().whileTrue(RunManipulatorCommand.withMode(intake, Manipulator.Modes.FORWARD));
    operator.x().whileTrue(RunManipulatorCommand.withMode(intake, Manipulator.Modes.REVERSE));

    // Intake Angler
    operator.y().onTrue(new RunAnglerCommand(() -> pivot.setpoint.plus(Rotation2d.fromDegrees(10)), pivot));
    operator.a().onFalse(new RunAnglerCommand(() -> pivot.setpoint.minus(Rotation2d.fromDegrees(10)), pivot));
    operator.povUp().onTrue(new RunAnglerCommand(() -> pivot.setpoint = (Constants.IntakeConstants.ampAngle), pivot));
    operator.leftBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.MAXIMUM, pivot));
    operator.rightBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.MAXIMUM, pivot));
    driver.stow().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.MAXIMUM, pivot));

    operator.leftTrigger().whileTrue(RunManipulatorCommand.withMode(intake, Manipulator.Modes.FORWARD));
    operator.x().whileTrue(RunManipulatorCommand.withMode(intake, Manipulator.Modes.REVERSE));
  }


  public Command getAutonomousCommand() {
    return autonMagic.procureAuton();
  }

  public void onTeleop() {
    pivot.setpoint = Rotation2d.fromRadians(pivot.absoluteEncoder.getPosition());
  }
}
