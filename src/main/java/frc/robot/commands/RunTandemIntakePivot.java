package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.team5431.titan.core.misc.Logger;

public class RunTandemIntakePivot extends Command {

  private final Intake intake;
  private final Angler pivot;
  private boolean noteStored;

  /**
   * @param intake intake subsystem
   * @param pivot pivot subsystem
   * 
   * Deploys intake pivot to ground, runs intake fowards, checks if game piece is detected, if so ends and stow
   * 
   */
  public RunTandemIntakePivot(Intake intake, Angler pivot) {
    this.intake = intake;
    this.pivot = pivot;
    this.noteStored = false;

    addRequirements(intake, pivot);
  }

  @Override
  public void initialize() {
    Logger.l("Starting intake tandem command");
    new RunAnglerCommand(RunAnglerCommand.AnglerModes.DEPLOY, pivot);
  }

  @Override
  public void execute() {
    new RunManipulatorCommand(intake, Manipulator.Modes.FORWARD);
    this.noteStored = intake.checkGamePieceStatus(); 
  }

  @Override
  public boolean isFinished(){
    return noteStored;
  }

  @Override
  public void end(boolean interrupted) {
    new RunAnglerCommand(RunAnglerCommand.AnglerModes.RETRACT, pivot);
    intake.stopNeutral();
    Logger.l("Ending tandem command");
    
  }
}
