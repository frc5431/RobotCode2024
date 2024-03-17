package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SimpleSpeaker extends SequentialCommandGroup {

    public SimpleSpeaker(Shooter shooter, Intake intake, Angler pivot) {
        addCommands(
            RunManipulatorCommand.withMode(intake, IntakeModes.INTAKE).withTimeout(0.3),
            new RunAnglerCommand(AnglerModes.STOW, pivot, TerminationCondition.SETPOINT_REACHED),
            new RunShooterCommand(shooter, ShooterModes.SpeakerShot).withTimeout(1),
            Commands.parallel(
                    new RunShooterCommand(shooter, ShooterModes.SpeakerShot),
                    RunManipulatorCommand.withMode(intake, IntakeModes.OUTAKE)).withTimeout(2));
    }
}
