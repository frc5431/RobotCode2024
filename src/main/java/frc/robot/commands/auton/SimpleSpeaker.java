package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunManipulatorCommand.ManipulatorMode;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;

public class SimpleSpeaker extends SequentialCommandGroup {
    
    public SimpleSpeaker(Manipulator intake, Manipulator shooter, Angler pivot) {

    addCommands(
        Commands.race(
        RunManipulatorCommand.withMode(intake, ManipulatorMode.FORWARD).withTimeout(0.1),
        RunManipulatorCommand.withMode(shooter, ManipulatorMode.REVERSE).withTimeout(4),
        new RunAnglerCommand(AnglerModes.MAXIMUM, pivot, TerminationCondition.SETPOINT_REACHED),
        RunManipulatorCommand.withMode(intake, ManipulatorMode.REVERSE).withTimeout(0.3)),
        RunManipulatorCommand.withPower(shooter, 0.1)
    );
    }
}
