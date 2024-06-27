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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AmperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.Constants.TunerConstatns;
import frc.robot.Constants.AmperConstants.AmperModes;
import frc.robot.commands.BlinkinStrobeCommand;
import frc.robot.commands.HandoffCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.commands.auton.AmpScore;
import frc.robot.commands.auton.AutoIntakeNote;
import frc.robot.commands.auton.DistantSpeakerScore;
import frc.robot.commands.auton.IntakeNote;
import frc.robot.commands.auton.SimpleSpeaker;
import frc.robot.subsystems.Amper;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.swerve.TitanFieldCentricFacingAngle;
import frc.team5431.titan.core.joysticks.CommandJoystick;
import frc.team5431.titan.core.joysticks.CommandLogitechExtreme3D;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.leds.BlinkinPattern;

public class RobotContainer {

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1); 
  public static final CommandXboxController stemGal = new CommandXboxController(0);
  private final Systems systems = new Systems();
  private final Drivebase drivebase = systems.getDrivebase();

  private final Pivot pivot = systems.getPivot();
  private final Pivot amperPivot = systems.getAmperPivot();
  private final Climber rightClimber = systems.getRightClimber();
  private final Climber leftClimber = systems.getLeftClimber();


  private final Intake intake = systems.getIntake();
  private final Amper amper = systems.getAmper();
  private final Shooter shooter = systems.getShooter();
  private final Blinkin blinkin = systems.getBlinkin();
  public SendableChooser<Boolean> isGals;
  private final AutonMagic autonMagic;
  private double vibrateTime = 0;
  private boolean shooterSpeed;
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



  // Control Schemes
  Trigger d_angleLock = driver.a();
  Trigger d_apriltagLock = driver.b();
  Trigger d_robotOriented = driver.rightBumper();
  Trigger d_resetGyro = driver.y();
  // Climber
  Trigger d_incrementClimber = driver.a();
  Trigger d_decrementRightClimber = driver.rightTrigger();
  Trigger d_decrementLeftClimber = driver.leftTrigger();

  Trigger d_rightClimber = driver.rightStick();
  Trigger d_leftClimber = driver.leftStick();

  Trigger d_stowIntake = driver.leftBumper();

  //https://www.padcrafter.com/?templates=Controller+Scheme+1&leftTrigger=Intake&rightTrigger=Speaker+Shot&leftBumper=Stow+Intake&rightBumper=Pivot+Automatic&leftStick=Pivot+Manual+Control&rightStick=Amper+Manual+Control&rightStickClick=Outtake+Amper&dpadUp=Deploy+Intake&dpadRight=Amper+Intake&dpadDown=Max+Power+Distant+Shot&yButton=Stage+Shot&xButton=Outtake&bButton=Reverse+Shooter&aButton=Distant+Shot&startButton=Deploy+Amper&backButton=Handoff
  // Shooter
  Trigger o_speakerShot = operator.rightTrigger();
  Trigger o_reverseShooter = operator.b();
  Trigger o_distantShot = operator.a();
  Trigger o_stageShot = operator.y();
  Trigger o_evilModeDistantShot = operator.povDown();
  // Pivot
  Trigger  o_deployIntake = operator.povUp();
  Trigger  o_pivotAutomatic = operator.rightBumper();
  Trigger  o_incrementPivot = operator.axisGreaterThan(1, 0.15);
  Trigger  o_decrementPivot = operator.axisGreaterThan(1, -0.15);
  Trigger  o_stow = operator.leftBumper();
  Trigger  o_downPivots = operator.povRight();
  // Intake
  Trigger  o_intake = operator.leftTrigger();
  Trigger  o_outtake = operator.x();
  // Amper
  Trigger  o_incrementAmper = operator.axisGreaterThan(5, 0.15);
  Trigger  o_decrementAmper = operator.axisLessThan(5, -0.15);
  // AmperPivot
  Trigger  o_timedOutake = operator.start();
  Trigger  o_deployAmper = operator.back(); // menu
  Trigger  o_outtakeAmper = operator.rightStick();
  Trigger  o_amperIntake = operator.leftStick();
  // Lights
  Trigger o_strobeLights = operator.povLeft();


  public RobotContainer() {
    NamedCommands.registerCommand("AmpScore", new AmpScore(shooter, intake));
    NamedCommands.registerCommand("ShooterRev", new StartEndCommand(() -> shooter.runPair(ShooterModes.SpeakerDistant.speed, shooter.distantTopController, shooter.distantBottomController), () -> {}).withTimeout(3));
    NamedCommands.registerCommand("SpeakerScore", new SimpleSpeaker(shooter, intake));
    NamedCommands.registerCommand("DistantSpeakerScore", new DistantSpeakerScore(shooter, intake));
    NamedCommands.registerCommand("GrabNote", new AutoIntakeNote(intake, pivot));

    autonMagic = new AutonMagic();


    // drivebase.seedField Relative();
    facingRequest.withPID(new PIDController(6, 0.01, 0.008));
    facingRequest.withDampening(new WeightedAverageController(45));
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
    if (angle.isPresent()) {
      SmartDashboard.putNumber("Target", angle.get().getRadians());
    }

    if (shooter.mode == ShooterModes.NONE) {
      pivot.setpoint = pivot.setpoint;
    } else if (shooter.mode.usesDistant) {
      var encoderPos = edu.wpi.first.math.util.Units.radiansToDegrees(pivot.absoluteEncoder.getPosition());
      if (encoderPos >= 200) {
        pivot.setpoint = Constants.IntakeConstants.anglerConstants.mainAngle;
        return;
      }
      pivot.setpoint = IntakeConstants.distantStowAngle;
      amperPivot.setpoint = AmperConstants.anglerConstants.minAngle;
      intakeNote.cancel();
    } else if (shooter.mode.usesMain) {
      pivot.setpoint = IntakeConstants.mainAngle;
      amperPivot.setpoint = AmperConstants.anglerConstants.minAngle;
      intakeNote.cancel();
    }

    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

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
    shooterSpeed = shooter.isClose(100);
    SmartDashboard.putBoolean("Shooter at Speed", shooterSpeed);
    operator.getHID().setRumble(RumbleType.kLeftRumble, (shooterSpeed) ? 0.5 : 0);
    if(shooterSpeed) {
      blinkin.set(BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    } 

  }
  
  private Command lockToAngleCommand(double redAngle, double blueAngle) {
    return drivebase.applyRequest(() -> {
      double u = driver.getLeftX();
      double v = driver.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
      return facingRequest
          .withVelocityX(
              modifyAxis(y2)
                  * TunerConstatns.kSpeedAt12VoltsMps)
          .withHeading(edu.wpi.first.math.util.Units.degreesToRadians((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? blueAngle : redAngle))
          .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps);
    }).until(() -> Math.abs(driver.getRawAxis(4)) > 0.15);
  }

  private void configureBindings() {

     if(Constants.isGals){
       drivebase.setDefaultCommand( // Drivetrain will execute this command periodically
        drivebase.applyRequest(() -> {
         double u = stemGal.getLeftX();
      double v = stemGal.getLeftY();

      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);


          return driveFC
              .withVelocityX(modifyAxis(y2)* TunerConstatns.kSpeedAt12VoltsMps)
              .withVelocityY(modifyAxis(x2) * TunerConstatns.kSpeedAt12VoltsMps)
              .withRotationalRate(
                  modifyAxis(stemGal.getRightX()) * TunerConstatns.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
        }));

    o_strobeLights = stemGal.povDown();

    // Shooter
    o_speakerShot = stemGal.rightTrigger();

    o_reverseShooter = stemGal.b();

    //o_deployIntake = stemGal.two();
    o_evilModeDistantShot = stemGal.a();
    o_distantShot = stemGal.x(); 

    o_outtake = stemGal.povUp();
    o_intake = stemGal.leftTrigger();

    d_resetGyro = stemGal.y();
    o_downPivots = stemGal.povRight();

    o_pivotAutomatic = stemGal.rightBumper();

    // Amper
    o_incrementAmper
        .whileTrue(new RunCommand(() -> amperPivot.increment(operator.getRightY() * 1), pivot).repeatedly());
    o_decrementAmper
        .whileTrue(new RunCommand(() -> amperPivot.increment(operator.getRightY() * 1), pivot).repeatedly());

   
    o_deployAmper = stemGal.povUp();
    o_outtakeAmper = stemGal.povDown();
    o_amperIntake = stemGal.leftTrigger();


    } else {

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

    d_angleLock.onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(27.80, 27.80));

    driver.povUp().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(180, 180));

    driver.povUpRight().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(225, 225));

    driver.povRight().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(270, 270));

    driver.povDownRight().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(315, 315));

    driver.povDown().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(0, 0));

    driver.povDownLeft().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(45, 45));

    driver.povLeft().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(90, 90));

    driver.povUpLeft().onTrue(new InstantCommand(() -> {
      facingRequest.pid.reset();
    })).toggleOnTrue(lockToAngleCommand(135, 135));

    d_apriltagLock.onTrue(new InstantCommand(() -> {
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

    d_robotOriented.whileTrue(drivebase.applyRequest(() -> {
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
    // Amper
    o_incrementAmper
        .whileTrue(new RunCommand(() -> amperPivot.increment(operator.getRightY() * 1), pivot).repeatedly());
    o_decrementAmper
        .whileTrue(new RunCommand(() -> amperPivot.increment(operator.getRightY() * 1), pivot).repeatedly());

    }
 

    blinkin.setDefaultCommand(new InstantCommand(() -> blinkin.set(BlinkinPattern.CP1_2_TWINKLES), blinkin));

    d_resetGyro.onTrue(new InstantCommand(() -> drivebase.resetGyro()));
    
    d_incrementClimber.whileTrue(rightClimber.increment(-2).repeatedly().alongWith(leftClimber.increment(-2).repeatedly()));
    d_decrementRightClimber.whileTrue(rightClimber.increment(2).repeatedly());
    d_decrementLeftClimber.whileTrue(leftClimber.increment(2).repeatedly());

    // d_rightClimber.whileTrue(rightClimber.increment(0.8).repeatedly());
    // d_leftClimber.whileTrue(leftClimber.increment(0.8).repeatedly());

    d_stowIntake.onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, pivot).alongWith(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, amperPivot)));

    o_strobeLights.whileTrue(new BlinkinStrobeCommand(systems.getBlinkin(), BlinkinPattern.ORANGE));
    // Shooter
    o_speakerShot.whileTrue(shooter.runShooterCommand(ShooterModes.SpeakerShot));
    o_reverseShooter.whileTrue(shooter.runShooterCommand(ShooterModes.REVERSE));
    o_distantShot.whileTrue(shooter.runShooterCommand(ShooterModes.SpeakerDistant));
    o_stageShot.whileTrue(shooter.runShooterCommand(ShooterModes.StageShot));
    // operator.start().whileTrue(shooter.runShooterCommand(ShooterModes.AmpShot));
    o_deployIntake.onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot));
    o_evilModeDistantShot.whileTrue(shooter.runShooterCommand(ShooterModes.DangerDistant));

    // Intake
    o_intake.whileTrue(RunManipulatorCommand.withMode(intake, IntakeModes.INTAKE).alongWith(amper.runPower(-0.3f)));
    o_outtake.whileTrue(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE).alongWith(amper.runPower(-1)));

    // Intake Angler
    o_incrementPivot
        .whileTrue(new RunCommand(() -> pivot.increment(-operator.getLeftY() * 1), pivot).repeatedly());
    o_decrementPivot
        .whileTrue(new RunCommand(() -> pivot.increment(-operator.getLeftY() * 1), pivot).repeatedly());

    o_stow.onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, pivot).alongWith(new RunAnglerCommand(AnglerModes.DEPLOY, amperPivot)));
    o_downPivots.onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot).alongWith(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, amperPivot, TerminationCondition.SETPOINT_REACHED)).andThen(new InstantCommand(() -> amperPivot.setpoint = Units.Degree.of(232))));
    o_pivotAutomatic.onTrue(intakeNote);

    o_timedOutake.onTrue(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE).withTimeout(1));
    o_deployAmper // deploy
        .onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, amperPivot));
    o_outtakeAmper.whileTrue(amper.runMode(AmperModes.OUTAKE).alongWith(RunManipulatorCommand.withMode(intake, IntakeModes.INTAKE)));
    o_amperIntake
        .whileTrue(amper.runMode(AmperModes.INTAKE).alongWith(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE)));
    

    }

  public Command getAutonomousCommand() {
    return autonMagic.procureAuton();
  }

  public void onTeleop() {
    amper.motor.getPIDController().setOutputRange(-1, 1);
    amper.motor.burnFlash();
    pivot.setpoint = Units.Radians.of(pivot.absoluteEncoder.getPosition());
    amperPivot.setpoint = (Constants.AmperConstants.anglerConstants.minAngle);
    rightClimber.relativeEncoder.setPosition(0);
    leftClimber.relativeEncoder.setPosition(0);
    //rightClimber.setpoint = 0;
    //leftClimber.setpoint = 0;
  }

}