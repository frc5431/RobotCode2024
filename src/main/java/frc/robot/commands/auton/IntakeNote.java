package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Intake;

public class IntakeNote extends SequentialCommandGroup {
    
  public IntakeNote(Intake intake, Angler pivot) {
    DigitalInput beambreak = intake.getBeamBreakStatus();
    addCommands(
      new RunAnglerCommand(AnglerModes.DEPLOY, pivot),
      intake.runMode(IntakeModes.INTAKE).until(beambreak::get),
      new RunAnglerCommand(AnglerModes.STOW, pivot)
      
      );
      
  }
}
