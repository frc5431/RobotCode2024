package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import java.util.function.Supplier;

public class RunAnglerCommand extends Command {

  final AnglerModes mode;
  final Angler angler;
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

  public RunAnglerCommand(AnglerModes mode, Angler angler) {
    this(mode, angler, TerminationCondition.IMMEDIATE);
  }

  public RunAnglerCommand(Supplier<Double> rotation, Angler angler) {
    this(rotation, angler, TerminationCondition.IMMEDIATE);
  }

  public RunAnglerCommand(AnglerModes mode, Angler angler, TerminationCondition terminationCondition) {
    this.mode = mode;
    this.angler = angler;
    rotation = null;
    this.terminationCondition = terminationCondition;
  }

  public RunAnglerCommand(Supplier<Double> rotation, Angler angler, TerminationCondition terminationCondition) {
    this.mode = AnglerModes.CUSTOM;
    this.angler = angler;
    this.rotation = rotation;
    this.terminationCondition = terminationCondition;
  }

  @Override
  public void initialize() {
    
  }

  public void execute() {
    if (AnglerModes.DEPLOY == mode) {
      angler.runToMin();
    } else if (AnglerModes.STOW == mode) {
      angler.runToMain();
    } else {
      angler.setRotation(this.rotation.get());
    }
  }

  @Override
  public boolean isFinished() {
    if(terminationCondition == TerminationCondition.IMMEDIATE) {
      return true;
    }

    return angler.isFinished();
  }
}
