package frc.robot.commands.auton;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveLockedRotCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LasaVision;

public class PickupNote extends ParallelCommandGroup {

        /**
         * @author Rubyboat1207
         * Autonomus Note collection, rather than the Teleop Command SmartIntakeNote,
         * this command guides the chassis twoards the note during autonomus
         */
public PickupNote(Drivebase drivebase, Intake intake, Angler angler) {
        LasaVision vision = LasaVision.getInstance();
        final DigitalInput beamBreak = intake.getBeamBreakStatus();

        addCommands(
                new ParallelCommandGroup(
                        new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, angler)
                                .andThen(RunManipulatorCommand.withPower(intake, -1)),
                        new ProxyCommand(() -> {
                            Optional<Translation2d> location = vision.getObjectLocation();
                            if (location.isEmpty()) {
                                return new InstantCommand();
                            }
                            Translation2d objectFieldPosition = location.get();
                            Pose2d robotFieldPosition = drivebase.getPose();
                            return new DriveLockedRotCommand(drivebase,
                                    Math.atan2(objectFieldPosition.getY() - robotFieldPosition.getY(),
                                            objectFieldPosition.getX() - robotFieldPosition.getX()),
                                    () -> false);
                        }),
                        new WaitCommand(0.4).andThen(new InstantCommand(() -> drivebase
                                .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityY(0.3)))))
                        .until(beamBreak::get)

        );
    }
}
