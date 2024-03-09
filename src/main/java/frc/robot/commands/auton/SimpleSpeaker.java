package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunManipulatorCommand.ManipulatorMode;
import frc.robot.subsystems.Manipulator;

public class SimpleSpeaker extends SequentialCommandGroup {
    
    public SimpleSpeaker(Manipulator intake, Manipulator shooter) {

    addCommands(
        Commands.race(
        RunManipulatorCommand.withMode(shooter, ManipulatorMode.REVERSE).withTimeout(2.5),
        RunManipulatorCommand.withMode(intake, ManipulatorMode.REVERSE).withTimeout(0.3))
    );
    }
}
