package frc.robot.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;
import java.util.function.Supplier;

public class RunAnglerCommand extends Command {

  final AnglerModes mode;
  final Pivot angler;
  Supplier<Double> rotation;

  public enum AnglerModes {
    STOW,
    DEPLOY,
    CUSTOM
  }

  public enum TerminationCondition {
    IMMEDIATE,
    SETPOINT_REACHED
  }

  final TerminationCondition terminationCondition;

  public RunAnglerCommand(AnglerModes mode, Pivot angler) {
    this(mode, angler, TerminationCondition.IMMEDIATE);
  }

  public RunAnglerCommand(Supplier<Double> rotation, Pivot angler) {
    this(rotation, angler, TerminationCondition.IMMEDIATE);
  }

  public RunAnglerCommand(AnglerModes mode, Pivot angler, TerminationCondition terminationCondition) {
    this.mode = mode;
    this.angler = angler;
    rotation = null;
    this.terminationCondition = terminationCondition;
  }

  public RunAnglerCommand(Supplier<Double> rotation, Pivot angler, TerminationCondition terminationCondition) {
    this.mode = AnglerModes.CUSTOM;
    this.angler = angler;
    this.rotation = rotation;
    this.terminationCondition = terminationCondition;
  }

  @Override
  public void initialize() {
    
  }

  boolean isIntermediate = false;

  public void execute() {
    if (AnglerModes.DEPLOY == mode) {
      if(angler.absoluteEncoder.getPosition() - Math.PI > 0) {
        isIntermediate = true;
        angler.setpoint = Constants.IntakeConstants.ampAngle;
        return;
      }
      isIntermediate = false;
      angler.runToMin();
    } else if (AnglerModes.STOW == mode) {
      isIntermediate = false;
      angler.runToMain();
    } else {
      angler.setRotation(this.rotation.get());
    }
  }

  @Override
  public boolean isFinished() {
    if(terminationCondition == TerminationCondition.IMMEDIATE && !isIntermediate) {
      return true;
    }

    return angler.isFinished();
  }
}
