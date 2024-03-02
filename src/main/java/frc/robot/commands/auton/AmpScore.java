package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.RunAnglerCommand;
import frc.robot.commands.RunManipulatorCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Manipulator;

public class AmpScore extends SequentialCommandGroup {
    
  private final Manipulator intake;
  private final Angler pivot;

  public AmpScore(Manipulator intake, Angler pivot) {
    this.intake = intake;
    this.pivot = pivot;

    addCommands(new RunAnglerCommand(() -> pivot.setpoint = (Constants.IntakeConstants.ampAngle), pivot)
    , new RunManipulatorCommand(intake, Manipulator.Modes.REVERSE).withTimeout(5));
  }
}
