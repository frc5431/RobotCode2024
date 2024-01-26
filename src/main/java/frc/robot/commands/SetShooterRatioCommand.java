package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class SetShooterRatioCommand extends Command {

  Shooter shooter;
  Supplier<Shooter.ShooterRatio> ratioSupplier;

  public SetShooterRatioCommand(Shooter shooter, Shooter.ShooterRatio ratio) {
    this(shooter, () -> ratio);
  }

  public SetShooterRatioCommand(Shooter shooter, Supplier<Shooter.ShooterRatio> ratioSupplier) {
    this.shooter = shooter;
    this.ratioSupplier = ratioSupplier;
  }

  @Override
  public void initialize() {
    shooter.setRatio(ratioSupplier.get());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
