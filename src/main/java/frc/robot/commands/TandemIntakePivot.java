package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.Systems;

import frc.team5431.titan.core.misc.Logger;

public class TandemIntakePivot extends Command {

  private final Systems system;
  private final Intake intake;
  private final Angler pivot;

  public TandemIntakePivot(Intake intake, Angler pivot) {
    this.intake = systems.getIntake();
    this.pivot = systems.getPivot();

    addRequirements(intake, pivot);
  }

  @Override
  public void initialize() {
    Logger.l("Starting intake pivot tandem command");
    
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    Logger.l("Ending tandem command");
    
  }
}
