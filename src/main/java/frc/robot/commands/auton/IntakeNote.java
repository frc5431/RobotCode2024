package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.Modes;

public class IntakeNote extends SequentialCommandGroup {
    
  private final Manipulator.Modes mode;

  public IntakeNote(Manipulator intake, Angler pivot) {
    this.mode = Modes.REVERSE;

    // addCommands(new RunAnglerCommand(RunAnglerCommand.AnglerModes.RETRACT, pivot)
    // , Commands.race(Commands.startEnd(() -> intake.run(mode), () -> intake.stopNeutral(), intake).withTimeout(2), ),
    // Commands.race(new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot), 
    // new RunManipulatorCommand(intake, Manipulator.Modes.FORWARD));
    
    //);
  }
}
