package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.commands.RunAnglerCommand.TerminationCondition;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Manipulator.Modes;

public class AmpScore extends SequentialCommandGroup {
    
  public AmpScore(Manipulator intake, Angler pivot) {

    addCommands(
      new RunAnglerCommand(() -> pivot.setpoint = (Constants.IntakeConstants.ampAngle), pivot, TerminationCondition.SETPOINT_REACHED),
      RunManipulatorCommand.withMode(intake, Modes.REVERSE).withTimeout(0.3)
    );
  }
}
