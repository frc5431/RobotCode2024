package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;

public class RunAnglerCommand extends Command {

  final AnglerModes mode;
  final Angler angler;
  Rotation2d rotation;

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

  public RunAnglerCommand(Angler angler, Rotation2d rotation) {
    this.mode = AnglerModes.CUSTOM;
    this.angler = angler;
    this.rotation = rotation;
  }

  @Override
  public void initialize() {
    if (AnglerModes.RETRACT == mode) {
      angler.retract();
    } else if (AnglerModes.DEPLOY == mode) {
      angler.deploy();
    } else {
      angler.setRotation(rotation);
    }
  }

  @Override
  public boolean isFinished() {
    return angler.isFinished(Units.degreesToRadians(5));
  }
}
