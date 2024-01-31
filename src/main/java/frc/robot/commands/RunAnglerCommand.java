package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import java.util.function.Supplier;

public class RunAnglerCommand extends Command {

  final AnglerModes mode;
  final Angler angler;
  Supplier<Rotation2d> rotation;

  public enum AnglerModes {
    DEPLOY,
    RETRACT,
    CUSTOM
  }

  public RunAnglerCommand(AnglerModes mode, Angler angler) {
    this.mode = mode;
    this.angler = angler;
    rotation = null;
  }

  public RunAnglerCommand(Supplier<Rotation2d> rotation, Angler angler) {
    this.mode = AnglerModes.CUSTOM;
    this.angler = angler;
    this.rotation = rotation;
  }

  @Override
  public void initialize() {
    
  }

  public void execute() {
    if (AnglerModes.RETRACT == mode) {
      angler.retract();
    } else if (AnglerModes.DEPLOY == mode) {
      angler.deploy();
    } else {
      angler.setRotation(this.rotation.get());
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
