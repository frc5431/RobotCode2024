package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.AnglerModes;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;

public class AutoIntakeNote extends SequentialCommandGroup {
  int positives = 0;
  DigitalInput beambreak;


  public AutoIntakeNote(Intake intake, Pivot pivot) {
    beambreak = intake.getBeamBreakStatus();
    addCommands(
        new InstantCommand(() -> {positives = 0;}),
        Commands.race(
            RunManipulatorCommand.withMode(intake, IntakeModes.INTAKE),
            Commands.sequence(
                new RunAnglerCommand(AnglerModes.DEPLOY, pivot),
                new WaitUntilCommand(() -> !beambreak.get() && positives > 10),
                new RunAnglerCommand(AnglerModes.STOW, pivot, TerminationCondition.SETPOINT_REACHED))).alongWith(new RunCommand(this::periodic)),
        RunManipulatorCommand.withMode(intake, IntakeModes.STOPPED).withTimeout(0.3)
      
    );

  }

  public void periodic() {
    if(beambreak.get()) {
      positives++;
    }
  }
}
