package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class RunManipulatorCommand extends Command {
  public enum ManipulatorMode {
    FORWARD,
    REVERSE,
    STOPPED,
  }

  // Subsystems
  private Manipulator manipulator;

  // Config
  private boolean usePower;
  private double power;
  private ManipulatorMode mode;

  public static RunManipulatorCommand withMode(Manipulator manipulator, ManipulatorMode mode) {
    RunManipulatorCommand cmd = new RunManipulatorCommand();

    cmd.manipulator = manipulator;
    cmd.mode = mode;
    cmd.usePower = false;

    return cmd;
  }

  public static RunManipulatorCommand withPower(Manipulator manipulator, double power) {
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
    manipulator.run(ManipulatorMode.STOPPED);
  }
}
