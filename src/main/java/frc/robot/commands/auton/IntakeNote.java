package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;

public class IntakeNote extends Command {
    
  private final Manipulator intake;
  private final Angler pivot;

  private boolean hasNote;

  public IntakeNote(Manipulator intake, Angler pivot, boolean hasNote) {
    this.intake = intake;
    this.pivot = pivot;
    this.hasNote = hasNote;
  }

  @Override
  public void execute() {
    new RunAnglerCommand(RunAnglerCommand.AnglerModes.RETRACT, pivot);
    new RunManipulatorCommand(intake, Manipulator.Modes.REVERSE);

    
    
  }

  @Override
  public void end(boolean interrupted) {



  }

}
