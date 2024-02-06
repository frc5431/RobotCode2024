package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Shooter;
import frc.team5431.titan.core.misc.Logger;

public class RunTandemShooterPivot extends Command {

  private final Shooter shooter;
  private final Angler pivot;

  /**
   * @param intake intake subsystem
   * @param pivot pivot subsystem
   * 
   * Deploys intake pivot to ground, runs intake fowards, checks if game piece is detected, if so ends and stow
   * 
   */
  public RunTandemShooterPivot(Shooter shooter, Angler pivot) {
    this.shooter = shooter;
    this.pivot = pivot;

    addRequirements(shooter, pivot);
  }

  @Override
  public void initialize() {
    Logger.l("Starting intake tandem command");
    new RunAnglerCommand(RunAnglerCommand.AnglerModes.RETRACT, pivot);
  }

  @Override
  public void execute() {
    new RunManipulatorCommand(shooter, 1);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.l("Ending tandem command");
    
  }
}
