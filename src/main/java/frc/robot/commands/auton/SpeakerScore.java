package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;

public class SpeakerScore extends SequentialCommandGroup {
    public SpeakerScore(Manipulator intake, Manipulator shooter, Angler angler, Angler pivot) {
        addCommands();
    }
}
