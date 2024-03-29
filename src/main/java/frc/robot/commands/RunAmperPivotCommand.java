package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmperPivot;
import java.util.function.Supplier;

public class RunAmperPivotCommand extends Command {

  final AnglerModes mode;
  final AmperPivot angler;
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

  public RunAmperPivotCommand(AnglerModes mode, AmperPivot angler) {
    this(mode, angler, TerminationCondition.IMMEDIATE);
  }

  public RunAmperPivotCommand(Supplier<Double> rotation, AmperPivot angler) {
    this(rotation, angler, TerminationCondition.IMMEDIATE);
  }

  public RunAmperPivotCommand(AnglerModes mode, AmperPivot angler, TerminationCondition terminationCondition) {
    this.mode = mode;
    this.angler = angler;
    rotation = null;
    this.terminationCondition = terminationCondition;
  }

  public RunAmperPivotCommand(Supplier<Double> rotation, AmperPivot angler, TerminationCondition terminationCondition) {
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
