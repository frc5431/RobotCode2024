package frc.robot.commands.groups;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveLockedRotCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PheonixDrivebase;
import frc.robot.subsystems.Manipulator.Modes;

public class ShootToPositionCommandGroup extends ParallelCommandGroup {

  public ShootToPositionCommandGroup(Angler angler, Manipulator shooter, PheonixDrivebase drivebase, Manipulator intake,
      Translation3d fieldPosition) {
    Pose2d position = drivebase.getOdometry().getEstimatedPosition();

    double theta = Math.atan2(position.getY() - fieldPosition.getY(), position.getX() - fieldPosition.getX());

    Optional<Double> shootAngle = Angler.calculateLaunchAngle(
        shooter.geConstants().estimatedImpulseForceMetersPerSecond,
        position.getTranslation().getDistance(new Translation2d(fieldPosition.getX(), fieldPosition.getY())),
        (double) fieldPosition.getZ());

    if (shootAngle.isEmpty()) {
      return;
    }

    addCommands(
        new DriveLockedRotCommand(drivebase, Units.radiansToDegrees(theta), () -> false).andThen(
            new RunAnglerCommand(() -> Rotation2d.fromDegrees(shootAngle.get()), angler,
                TerminationCondition.SETPOINT_REACHED)),
        RunManipulatorCommand.withMode(shooter, Modes.FORWARD).withTimeout(5));
  }
}
