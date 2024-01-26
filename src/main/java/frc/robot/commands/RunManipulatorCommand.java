package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

public class RunManipulatorCommand extends Command {
    // Subsystems
    private final Manipulator manipulator;
    
    // Config
    private boolean usePower;
    private double power;
    private Manipulator.Modes mode;
    private boolean endStopped;

    public RunManipulatorCommand(Manipulator manipulator, Manipulator.Modes mode, boolean endStopped) {
        this(manipulator, endStopped);
        this.usePower = false;
        this.mode = mode;
    }

    public RunManipulatorCommand(Manipulator manipulator, double power, boolean endStopped) {
        this(manipulator, endStopped);
        this.usePower = true;
        this.power = power;
    }

    public RunManipulatorCommand(Manipulator manipulator, double power) {
        this(manipulator, power, true);
    }

    public RunManipulatorCommand(Manipulator manipulator, Manipulator.Modes mode) {
        this(manipulator, mode, true);
    }
    
    private RunManipulatorCommand(Manipulator manipulator, boolean endStopped) {
        this.manipulator = manipulator;
        addRequirements(manipulator);

        this.endStopped = endStopped;
    }


    @Override
    public void initialize() {
        if(usePower) {
            manipulator.runWithPower(power);
        }else {
            manipulator.run(mode);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(endStopped) {
            manipulator.run(Manipulator.Modes.STOPPED);
        }else {
            manipulator.run(Manipulator.Modes.NEUTRAL);
        }
        
    }
}
