package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterModes;
import frc.robot.subsystems.Shooter;

public class RunShooterCommand extends Command {

  private Shooter shooter;
  private ShooterModes mode;

  public RunShooterCommand(Shooter shooter, ShooterModes mode) {
    this.shooter = shooter;
    this.mode = mode;
  }

  @Override
  public void initialize() {
    shooter.RunShooter(mode);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopNeutral();
  }
}
