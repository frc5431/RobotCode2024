package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RunElevatorCommand extends Command{
  private Elevator elevator;

  public RunElevatorCommand (Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    elevator.setPosition();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
