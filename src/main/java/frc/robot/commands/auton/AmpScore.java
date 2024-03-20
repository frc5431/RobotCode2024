package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants.IntakeModes;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;

public class AmpScore extends SequentialCommandGroup {
    
  public AmpScore(Intake intake, Pivot pivot) {

    addCommands(
      new RunCommand( () -> pivot.setRotation(Constants.IntakeConstants.ampAngle), pivot).until(pivot::isFinished),
      RunManipulatorCommand.withMode(intake, IntakeModes.INTAKE).withTimeout(0.3)
    );
  }
}
