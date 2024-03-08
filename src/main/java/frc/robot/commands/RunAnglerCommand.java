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
    MAXIMUM,
    MINIMUM,
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

  public RunAnglerCommand(Supplier<Rotation2d> rotation, Angler angler) {
    this(rotation, angler, TerminationCondition.IMMEDIATE);
  }

  public RunAnglerCommand(AnglerModes mode, Angler angler, TerminationCondition terminationCondition) {
    this.mode = mode;
    this.angler = angler;
    rotation = null;
    this.terminationCondition = terminationCondition;
  }

  public RunAnglerCommand(Supplier<Rotation2d> rotation, Angler angler, TerminationCondition terminationCondition) {
    this.mode = AnglerModes.CUSTOM;
    this.angler = angler;
    this.rotation = rotation;
    this.terminationCondition = terminationCondition;
  }

  @Override
  public void initialize() {
    
  }

  public void execute() {
    if (AnglerModes.MINIMUM == mode) {
      angler.runToMin();
    } else if (AnglerModes.MAXIMUM == mode) {
      angler.runToMax();
    } else {
      angler.setRotation(this.rotation.get());
    }
  }

  @Override
  public boolean isFinished() {
    if(terminationCondition == TerminationCondition.IMMEDIATE) {
      return true;
    }

    return angler.isFinished(0.3);
  }
}
