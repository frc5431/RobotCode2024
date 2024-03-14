package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunManipulatorCommand extends Command {
  public enum IntakeModes {
    OUTAKE,
    INTAKE,
    STOPPED,
  }

  // Subsystems
  private Intake manipulator;

  // Config
  private boolean usePower;
  private double power;
  private IntakeModes mode;

  public static RunManipulatorCommand withMode(Intake manipulator, IntakeModes mode) {
    RunManipulatorCommand cmd = new RunManipulatorCommand();

    cmd.manipulator = manipulator;
    cmd.mode = mode;
    cmd.usePower = false;

    return cmd;
  }

  public static RunManipulatorCommand withPower(Intake manipulator, double power) {
    RunManipulatorCommand cmd = new RunManipulatorCommand();

    cmd.manipulator = manipulator;
    cmd.power = power;
    cmd.usePower = true;

    return cmd;
  }

  @Override
  public void initialize() {
    if (usePower) {
      manipulator.runWithPower(power);
    } else {
      manipulator.run(mode);
    }
    

  }

  @Override
  public void end(boolean interrupted) {
    manipulator.run(IntakeModes.STOPPED);
  }
}
