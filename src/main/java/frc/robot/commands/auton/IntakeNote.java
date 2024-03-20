package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;

public class IntakeNote extends SequentialCommandGroup {

  public IntakeNote(Intake intake, Pivot pivot) {
    DigitalInput beambreak = intake.getBeamBreakStatus();
    addCommands(
      Commands.race(
        intake.runMode(IntakeModes.INTAKE),
        new RunAnglerCommand(AnglerModes.DEPLOY, pivot, TerminationCondition.SETPOINT_REACHED)
        .alongWith(new WaitUntilCommand(() -> beambreak.get())
          .andThen(new RunAnglerCommand(AnglerModes.STOW, pivot, TerminationCondition.SETPOINT_REACHED))
        )
      )
    );

  }
}
