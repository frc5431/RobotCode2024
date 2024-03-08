package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunManipulatorCommand.ManipulatorMode;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;
import frc.robot.Systems;

public class IntakeNote extends SequentialCommandGroup {
    
  private Systems systems;

  public IntakeNote(Manipulator intake, Angler pivot) {
    addCommands(
      RunManipulatorCommand.withMode(intake, ManipulatorMode.REVERSE),
      new RunAnglerCommand(AnglerModes.MINIMUM, pivot, TerminationCondition.SETPOINT_REACHED),

      new RunAnglerCommand(AnglerModes.MAXIMUM, pivot, TerminationCondition.SETPOINT_REACHED)
    );
    
  }
}
