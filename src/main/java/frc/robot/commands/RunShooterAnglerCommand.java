package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.ShooterAngler;

import java.util.function.Supplier;

public class RunShooterAnglerCommand extends Command {

  final ShooterAnglerModes mode;
  final ShooterAngler angler;
  Supplier<Double> rotation;

  public enum ShooterAnglerModes {
    MAXIMUM,
    MINIMUM,
    CUSTOM
  }

  public enum TerminationCondition {
    IMMEDIATE,
    SETPOINT_REACHED
  }

  final TerminationCondition terminationCondition;

  public RunShooterAnglerCommand(ShooterAnglerModes mode, ShooterAngler angler) {
    this(mode, angler, TerminationCondition.IMMEDIATE);
  }

  public RunShooterAnglerCommand(Supplier<Double> rotation, ShooterAngler angler) {
    this(rotation, angler, TerminationCondition.IMMEDIATE);
  }

  public RunShooterAnglerCommand(ShooterAnglerModes mode, ShooterAngler angler, TerminationCondition terminationCondition) {
    this.mode = mode;
    this.angler = angler;
    rotation = null;
    this.terminationCondition = terminationCondition;
  }

  public RunShooterAnglerCommand(Supplier<Double> rotation, ShooterAngler angler, TerminationCondition terminationCondition) {
    this.mode = ShooterAnglerModes.CUSTOM;
    this.angler = angler;
    this.rotation = rotation;
    this.terminationCondition = terminationCondition;
  }

  @Override
  public void initialize() {
    
  }

  public void execute() {
    if (ShooterAnglerModes.MINIMUM == mode) {
      angler.runToMin();
    } else if (ShooterAnglerModes.MAXIMUM == mode) {
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
