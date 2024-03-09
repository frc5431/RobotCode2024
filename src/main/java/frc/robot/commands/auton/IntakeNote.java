package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunManipulatorCommand.ManipulatorMode;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;

public class IntakeNote extends SequentialCommandGroup {
    
  public IntakeNote(Manipulator intake, Angler pivot, DigitalInput bb) {
    addCommands(
      Commands.parallel(
      RunManipulatorCommand.withMode(intake, ManipulatorMode.REVERSE).until(bb::get),
      new RunAnglerCommand(AnglerModes.MINIMUM, pivot, TerminationCondition.SETPOINT_REACHED)),
      new RunAnglerCommand(AnglerModes.MAXIMUM, pivot, TerminationCondition.SETPOINT_REACHED)
    );
    
  }
}
