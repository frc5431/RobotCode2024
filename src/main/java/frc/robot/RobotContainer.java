// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.TunerConstatns;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PheonixDrivebase;
import frc.robot.subsystems.Vision;
import frc.team5431.titan.core.joysticks.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  private final Systems systems = new Systems();
  private final PheonixDrivebase drivebase = systems.getDrivebase();
  private final Vision vision = systems.getVision();
  private final Angler pivot = systems.getPivot();
  private final Manipulator intake = systems.getIntake();
  private final Manipulator shooter = systems.getShooter();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      //.withDeadband(TunerConstatns.kSpeedAt12VoltsMps * 0.2).withRotationalDeadband(TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {
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

  private void configureBindings() {
    shooter.setRatio(Constants.ShooterConstants.shooterRatio);

    driver.setDeadzone(0.15);
/* 
    // drivebase.setDefaultCommand(
    //     new DefaultDriveCommand(
    //         systems,
    //         (Supplier<Pair<Double, Double>>) () -> {
    //           double inX = -driver.getLeftY(); // swap intended
    //           if(driver.povUp().getAsBoolean()) {
    //             inX = 0.3;
    //           }
    //           double inY = -driver.getLeftX();
    //           double mag = Math.hypot(inX, inY);
    //           double theta = Math.atan2(inY, inX);
    //           return Pair.of(modifyAxis(mag) * Drivebase.MAX_VELOCITY_METERS_PER_SECOND, theta);
    //         },
    //         () -> -modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));*/
    drivebase.setDefaultCommand( // Drivetrain will execute this command periodically 
        drivebase.applyRequest(() -> drive.withVelocityX(-modifyAxis(driver.getLeftY()) * TunerConstatns.kSpeedAt12VoltsMps) // Drive forward with
                                                                              // negative Y (forward)
            .withVelocityY(-modifyAxis(driver.getLeftX()) * TunerConstatns.kSpeedAt12VoltsMps) // Drive left with negative X (left)
            .withRotationalRate(-modifyAxis(driver.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND) // Drive counterclockwise with negative X (left)
        ));
    
    driver.y().onTrue(new InstantCommand(() -> drivebase.zeroGyro()));

    SmartDashboard.putNumber("turn axis", -modifyAxis(-driver.getRightX()) * Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    //STUPID STUFF FOR PHILLIP, DELETE
    // Shooter
    driver.b().whileTrue(new RunManipulatorCommand(shooter, -1));
    driver.rightTrigger().whileTrue(new RunManipulatorCommand(shooter, 1));

    // Intake
    driver.leftTrigger().whileTrue(new RunManipulatorCommand(intake, Manipulator.Modes.FORWARD));
    driver.x().whileTrue(new RunManipulatorCommand(intake, Manipulator.Modes.REVERSE));
 
    // Intake Angler
    driver.povLeft().onTrue(new RunAnglerCommand(() -> pivot.setpoint.plus(Rotation2d.fromDegrees(10)), pivot));
    driver.povRight().onFalse(new RunAnglerCommand(() -> pivot.setpoint.minus(Rotation2d.fromDegrees(10)), pivot));
    driver.povUp().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.AMP, pivot));
    
    driver.leftBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot));
    driver.rightBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.RETRACT, pivot));







    // Shooter
    operator.b().whileTrue(new RunManipulatorCommand(shooter, -1));
    operator.rightTrigger().whileTrue(new RunManipulatorCommand(shooter, 1));

    // Intake
    operator.leftTrigger().whileTrue(new RunManipulatorCommand(intake, Manipulator.Modes.FORWARD));
    operator.x().whileTrue(new RunManipulatorCommand(intake, Manipulator.Modes.REVERSE));
 
    // Intake Angler
    operator.y().onTrue(new RunAnglerCommand(() -> pivot.setpoint.plus(Rotation2d.fromDegrees(10)), pivot));
    operator.a().onFalse(new RunAnglerCommand(() -> pivot.setpoint.minus(Rotation2d.fromDegrees(10)), pivot));
    operator.povUp().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.AMP, pivot));
    
    operator.leftBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot));
    operator.rightBumper().onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.RETRACT, pivot));

  }

  public Command getAutonomousCommand() {
    return null;
  }

  public void onTeleop() {
    pivot.setpoint = Rotation2d.fromRadians(pivot.absoluteEncoder.getPosition());
  }
}
