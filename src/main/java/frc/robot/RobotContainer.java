// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AmperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.Constants.AmperConstants.AmperModes;
import frc.robot.commands.BlinkinStrobeCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.commands.auton.AmpScore;
import frc.robot.commands.auton.AutoIntakeNote;
import frc.robot.commands.auton.DistantSpeakerScore;
import frc.robot.commands.auton.IntakeNote;
import static edu.wpi.first.units.Units.*;
import frc.robot.commands.auton.SimpleSpeaker;
import frc.robot.subsystems.Amper;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
//import frc.robot.swerve.TitanFieldCentricFacingAngle;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import frc.team5431.titan.core.leds.Blinkin;
import frc.team5431.titan.core.leds.BlinkinPattern;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class RobotContainer {

  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  public static final CommandXboxController stemGal = new CommandXboxController(0);
  private final Systems systems = new Systems();
  private final Drivebase drivebase = systems.getDrivebase();

  private final Pivot pivot = systems.getPivot();
  private final Pivot amperPivot = systems.getAmperPivot();
  // private final Climber rightClimber = systems.getRightClimber();
  // private final Climber leftClimber = systems.getLeftClimber();

  private final Intake intake = systems.getIntake();
  private final Amper amper = systems.getAmper();
  private final Shooter shooter = systems.getShooter();
  private final Blinkin blinkin = systems.getBlinkin();
  private final AutonMagic autonMagic;
  private double vibrateTime = 0;
  private boolean shooterSpeed;
  private boolean shouldBeVibrating = false;
  private boolean oldBeamBreakStatus = true;
  double unchangedBeamBreakTime = 0;
  public Rotation2d targetRotation;
  // TitanFieldCentricFacingAngle facingRequest = new
  // TitanFieldCentricFacingAngle();

  IntakeNote intakeNote = new IntakeNote(intake, pivot);
  SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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

  // https://www.padcrafter.com/?templates=Controller+Scheme+1&leftTrigger=Intake&rightTrigger=Speaker+Shot&leftBumper=Stow+Intake&rightBumper=Pivot+Automatic&leftStick=Pivot+Manual+Control&rightStick=Amper+Manual+Control&rightStickClick=Outtake+Amper&dpadUp=Deploy+Intake&dpadRight=Amper+Intake&dpadDown=Max+Power+Distant+Shot&yButton=Stage+Shot&xButton=Outtake&bButton=Reverse+Shooter&aButton=Distant+Shot&startButton=Deploy+Amper&backButton=Handoff
  // Shooter
  Trigger o_speakerShot = operator.rightTrigger();
  Trigger o_reverseShooter = operator.b();
  Trigger o_distantShot = operator.a();
  Trigger o_stageShot = operator.y();
  Trigger o_evilModeDistantShot = operator.povDown();
  // Pivot
  Trigger o_deployIntake = operator.povUp();
  Trigger o_pivotAutomatic = operator.rightBumper();
  Trigger o_incrementPivot = operator.axisGreaterThan(1, 0.15);
  Trigger o_decrementPivot = operator.axisGreaterThan(1, -0.15);
  Trigger o_stow = operator.leftBumper();
  Trigger o_downPivots = operator.povRight();
  // Intake
  Trigger o_intake = operator.leftTrigger();
  Trigger o_outtake = operator.x();
  // Amper
  Trigger o_incrementAmper = operator.axisGreaterThan(5, 0.15);
  Trigger o_decrementAmper = operator.axisLessThan(5, -0.15);
  // AmperPivot
  Trigger o_timedOutake = operator.start();
  Trigger o_deployAmper = operator.back(); // menu
  Trigger o_outtakeAmper = operator.rightStick();
  Trigger o_amperIntake = operator.leftStick();
  // Lights
  Trigger o_strobeLights = operator.povLeft();

  public RobotContainer() {
    NamedCommands.registerCommand("AmpScore", new AmpScore(shooter, intake));
    NamedCommands.registerCommand("ShooterRev",
        new StartEndCommand(() -> shooter.runPair(ShooterModes.SpeakerDistant.speed, shooter.distantTopController,
            shooter.distantBottomController), () -> {
            }).withTimeout(3));
    NamedCommands.registerCommand("SpeakerScore", new SimpleSpeaker(shooter, intake));
    NamedCommands.registerCommand("DistantSpeakerScore", new DistantSpeakerScore(shooter, intake));
    NamedCommands.registerCommand("GrabNote", new AutoIntakeNote(intake, pivot));

    autonMagic = new AutonMagic();

    // drivebase.seedField Relative();
    // facingRequest.withPID(new PIDController(6, 0.01, 0.008));
    // facingRequest.withDampening(new WeightedAverageController(45));
    // facingRequest.gyro = drivebase.getGyro();
    configureBindings();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void periodic() {

    if (shooter.mode == ShooterModes.NONE) {
      pivot.setpoint = pivot.setpoint;
    } else if (shooter.mode.usesDistant) {
      var encoderPos = Units.Rotations.of(pivot.absoluteEncoder.getPosition()).in(Units.Degrees);
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
    if (shooterSpeed) {
      blinkin.set(BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
    }

  }

  public double deadzone(double input) {
    if(Math.abs(input) < 0.15) {
      return 0;
    }
    return input;
  }

  private void configureBindings() {

    driver.setDeadzone(0.15);

    drivebase.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivebase.applyRequest(() ->
          drive.withVelocityX(deadzone(-driver.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(deadzone(-driver.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(deadzone(-driver.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
  );

    // Amper
    o_incrementAmper
        .whileTrue(new RunCommand(() -> amperPivot.increment(operator.getRightY() * 0.1), pivot).repeatedly());
    o_decrementAmper
        .whileTrue(new RunCommand(() -> amperPivot.increment(operator.getRightY() * 0.1), pivot).repeatedly());

    blinkin.setDefaultCommand(new InstantCommand(() -> blinkin.set(BlinkinPattern.CP1_2_TWINKLES), blinkin));

    d_resetGyro.onTrue(new InstantCommand(() -> drivebase.resetGyro()));

    d_stowIntake.onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, pivot)
        .alongWith(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, amperPivot)));

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

    o_stow.onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, pivot)
        .alongWith(new RunAnglerCommand(AnglerModes.DEPLOY, amperPivot)));
    o_downPivots.onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot)
        .alongWith(
            new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, amperPivot, TerminationCondition.SETPOINT_REACHED))
        .andThen(new InstantCommand(() -> amperPivot.setpoint = Angle.ofRelativeUnits(232, Degree))));
    o_pivotAutomatic.onTrue(intakeNote);

    o_timedOutake.onTrue(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE).withTimeout(1));
    o_deployAmper // deploy
        .onTrue(new RunAnglerCommand(RunAnglerCommand.AnglerModes.STOW, amperPivot));
    o_outtakeAmper.whileTrue(
        amper.runMode(AmperModes.OUTAKE).alongWith(RunManipulatorCommand.withMode(intake, IntakeModes.INTAKE)));
    o_amperIntake
        .whileTrue(
            amper.runMode(AmperModes.INTAKE).alongWith(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE)));

  }

  public Command getAutonomousCommand() {
    return autonMagic.procureAuton();
  }

  public void onTeleop() {
    //pivot.setpoint = pivot.absoluteEncoder.getPosition();
    //amperPivot.setpoint = (Constants.AmperConstants.anglerConstants.minAngle);
  }

}