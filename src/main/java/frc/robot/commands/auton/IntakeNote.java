package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.Modes;

public class IntakeNote extends SequentialCommandGroup {
    

  public IntakeNote(Manipulator intake, Angler pivot) {
    addCommands(
      new RunAnglerCommand(AnglerModes.MINIMUM, pivot, TerminationCondition.SETPOINT_REACHED),
      RunManipulatorCommand.withMode(intake, Modes.REVERSE).withTimeout(0.3),
      new RunAnglerCommand(AnglerModes.MAXIMUM, pivot, TerminationCondition.SETPOINT_REACHED)
    );
    
  }
}
