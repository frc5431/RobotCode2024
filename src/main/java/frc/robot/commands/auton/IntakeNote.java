package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;

public class IntakeNote extends SequentialCommandGroup {

  public IntakeNote(Intake intake, Pivot pivot) {
    DigitalInput beambreak = intake.getBeamBreakStatus();
    addCommands(
      new RunAnglerCommand(AnglerModes.DEPLOY, pivot),
      new WaitUntilCommand(() -> !beambreak.get()),
      new RunAnglerCommand(AnglerModes.STOW, pivot, TerminationCondition.SETPOINT_REACHED)
    );
  }
}
