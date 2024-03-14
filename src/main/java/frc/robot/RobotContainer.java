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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.TunerConstatns;
import frc.robot.Constants.ShooterConstants.ShooterMode;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunClimberCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.commands.RunManipulatorCommand.ManipulatorMode;
import frc.robot.commands.auton.AmpScore;
import frc.robot.commands.auton.SimpleSpeaker;
import frc.robot.controllers.DriverController;
import frc.robot.controllers.DriverSkyflyController;
import frc.robot.controllers.DriverXboxController;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {
  private final DriverController driver;

  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Systems systems = new Systems();
  private final Drivebase drivebase = systems.getDrivebase();

  private final Angler pivot = systems.getPivot();
  private final Climber climber = systems.getClimber();

  private final Intake intake = systems.getIntake();
  private final Shooter shooter = systems.getShooter();
  private final AutonMagic autonMagic;

  private SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {
    NamedCommands.registerCommand("AmpScore", new AmpScore(intake, pivot));
    NamedCommands.registerCommand("SpeakerScore", new SimpleSpeaker(intake, shooter, pivot));
    NamedCommands.registerCommand("ZeroGyro", new InstantCommand(() -> drivebase.zeroGyro()));
    NamedCommands.registerCommand("PutDown", new RunAnglerCommand(() -> pivot.setpoint = (Constants.IntakeConstants.ampAngle), pivot, TerminationCondition.SETPOINT_REACHED).andThen(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot)));
    NamedCommands.registerCommand("90Gyro", new InstantCommand(() -> drivebase.set(-90)));



    autonMagic = new AutonMagic(systems);


    if(Constants.useXboxController) {
      driver = new DriverXboxController();
    }else {
      driver = new DriverSkyflyController();
    }

    drivebase.seedFieldRelative();
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

    drivebase.setDefaultCommand( // Drivetrain will execute this command periodically
        drivebase.applyRequest(() -> {
          // var axis = FGSquircularMap(driver.getLeftX() * .89, driver.getLeftY() * 0.89);
          return driveFC.withVelocityX(modifyAxis(driver.getLeftY() + (driver.temp_getController().povUp().getAsBoolean() ? 0.1 : 0)) * TunerConstatns.kSpeedAt12VoltsMps)
            .withVelocityY(modifyAxis(driver.getLeftX()) * TunerConstatns.kSpeedAt12VoltsMps)
            .withRotationalRate(
                -modifyAxis(driver.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        }));

    driver.resetGyro().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));
    if(driver instanceof DriverSkyflyController) {
      driver.resetGyro().onFalse(new InstantCommand(() -> drivebase.zeroGyro()));
    }

   driver.temp_getController().rightBumper().onTrue(new RunClimberCommand(climber, RunClimberCommand.ClimberMode.EXTENDED));
   driver.temp_getController().leftBumper().onTrue(new RunClimberCommand(climber, RunClimberCommand.ClimberMode.RETRACTED));
   //driver.temp_getController().leftTrigger().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.MAXIMUM, pivot));


    // Shooter
    operator.rightTrigger().whileTrue(shooter.speakerShot());
    operator.b().whileTrue(shooter.mainReverse());
    operator.a().whileTrue(shooter.stageShot());

    // Intake
    operator.leftTrigger().whileTrue(RunManipulatorCommand.withPower(intake, 1));
    operator.x().whileTrue(RunManipulatorCommand.withMode(intake, ManipulatorMode.OUTAKE));

    //Intake Angler
    operator.axisGreaterThan(1, 0.15).whileTrue(new RunAnglerCommand(() -> pivot.setpoint.plus(Rotation2d.fromDegrees(2)), pivot));
    operator.axisLessThan(1, -0.15).whileTrue(new RunAnglerCommand(() -> pivot.setpoint.minus(Rotation2d.fromDegrees(2)), pivot));

    operator.povRight().onTrue(new RunAnglerCommand(() -> pivot.setpoint = (Constants.IntakeConstants.ampAngle), pivot));  
    operator.leftBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, pivot));
    operator.rightBumper().onTrue(new RunAnglerCommand(() -> pivot.setpoint = (Constants.IntakeConstants.ampAngle), pivot, TerminationCondition.SETPOINT_REACHED).andThen(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot)));
    operator.leftStick().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot));

  }

  public Command getAutonomousCommand() {
    return autonMagic.procureAuton();
  }

  public void onTeleop() {
    pivot.setpoint = Rotation2d.fromRadians(pivot.absoluteEncoder.getPosition());
    if(shooter.getMode() == ShooterMode.AmpShot || shooter.getMode() == ShooterMode.SpeakerShot ||
      shooter.getMode() == ShooterMode.StageShot || shooter.getMode() == ShooterMode.DistantIn) {
      
    }
  }

}