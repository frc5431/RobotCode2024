package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import java.util.function.Supplier;

public class SetShooterRatioCommand extends Command {

  Manipulator shooter;
  Supplier<Manipulator.MotorRatio> ratioSupplier;

  public SetShooterRatioCommand(Manipulator shooter, Manipulator.MotorRatio ratio) {
    this(shooter, () -> ratio);
  }

  public SetShooterRatioCommand(Manipulator shooter, Supplier<Manipulator.MotorRatio> ratioSupplier) {
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
