package frc.robot.commands.auton;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveLockedRotCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PheonixDrivebase;

public class SpeakerScore extends SequentialCommandGroup {
    public SpeakerScore(Manipulator intake, Manipulator shooter, Angler angler, Angler pivot,
            PheonixDrivebase drivebase) {

        
        Alliance alliance = DriverStation.getAlliance().get();
        try {
            var tags = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            Optional<Double> angle = Angler.calculateLaunchAngle(
                    Constants.ShooterConstants.manipulatorConstants.estimatedImpulseForceMetersPerSecond, 0, 0);

            if (angle.isEmpty()) {
                return;
            }



            addCommands(Commands.parallel(new RunAnglerCommand(
                    () -> Rotation2d.fromDegrees(angle.get()),
                    angler, TerminationCondition.SETPOINT_REACHED),
                    new DriveLockedRotCommand(drivebase, 0, () -> false)));

        } catch (Exception ignored) {

        }

    }
}
