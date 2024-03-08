package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class RunManipulatorCommand extends Command {

  // Subsystems
  private Manipulator manipulator;

  // Config
  private boolean usePower;
  private double power;
  private Manipulator.Modes mode;

  public static RunManipulatorCommand withMode(Manipulator manipulator, Manipulator.Modes mode) {
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

  // public RunManipulatorCommand(Manipulator manipulator, Manipulator.Modes mode) {
  //   this(manipulator);
  //   this.usePower = false;
  //   this.mode = mode;
  // }

  // public RunManipulatorCommand(Manipulator manipulator, double power) {
  //   this(manipulator);
  //   this.usePower = true;
  //   this.power = power;
  // }

  // private RunManipulatorCommand(Manipulator manipulator) {
  //   this.manipulator = manipulator;
  //   addRequirements(manipulator);
  // }

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
    manipulator.run(Manipulator.Modes.STOPPED);
  }
}
