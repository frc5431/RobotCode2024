package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DistantSpeakerScore extends SequentialCommandGroup {

    public DistantSpeakerScore(Shooter shooter, Intake intake) {
        addCommands(
                Commands.parallel(
                        new RunShooterCommand(shooter, ShooterModes.SpeakerDistant),
                        new WaitCommand(2.25)
                                .andThen(RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE).withTimeout(0.5)))
                        .withTimeout(2.8));
    }
}
