package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.Supplier;

public class SetShooterRatioCommand extends Command {

  Intake shooter;
  Supplier<Intake.MotorRatio> ratioSupplier;

  public SetShooterRatioCommand(Intake shooter, Intake.MotorRatio ratio) {
    this(shooter, () -> ratio);
  }

  public SetShooterRatioCommand(Intake shooter, Supplier<Intake.MotorRatio> ratioSupplier) {
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
