package frc.robot.commands.auton;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveLockedRotCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.Modes;

public class SpeakerScore extends SequentialCommandGroup {
    public SpeakerScore(Manipulator intake, Manipulator shooter, Angler angler, Angler pivot,
            Drivebase drivebase) {

        Alliance alliance = DriverStation.getAlliance().get();
        try {
            var tags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            Optional<Double> angle = Angler.calculateLaunchAngle(
                    Constants.ShooterConstants.manipulatorConstants.estimatedImpulseForceMetersPerSecond, 0, 0);

            if (angle.isEmpty()) {
                return;
            }

            Pose3d targetPosition = tags.getTagPose(alliance == Alliance.Blue ? 7 : 4).get();

            targetPosition = new Pose3d(targetPosition.getX(), targetPosition.getY(),
                    targetPosition.getZ() + Units.feetToMeters(1) + Units.inchesToMeters(5),
                    targetPosition.getRotation());

            Pose2d drivebasePosition = drivebase.getOdometry().getEstimatedPosition();

            double drivebaseAngle = Math.atan2(drivebasePosition.getY() - targetPosition.getY(),
                    drivebasePosition.getX() - targetPosition.getX());

            addCommands(
                    Commands.parallel(
                            new RunAnglerCommand(
                                    () -> Rotation2d.fromDegrees(angle.get()),
                                    angler, TerminationCondition.SETPOINT_REACHED),
                            new DriveLockedRotCommand(drivebase, drivebaseAngle, () -> false)),
                    RunManipulatorCommand.withMode(shooter, Modes.FORWARD).withTimeout(1),
                    Commands.parallel(RunManipulatorCommand.withMode(intake, Modes.FORWARD).withTimeout(2),
                            RunManipulatorCommand.withMode(shooter, Modes.FORWARD).withTimeout(3)));

        } catch (Exception ignored) {

        }

    }
}
