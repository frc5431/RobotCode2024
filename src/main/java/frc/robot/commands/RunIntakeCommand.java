package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntakeCommand extends Command {
    
    final boolean forward;
    final Intake intake;

    @Override
    public void initialize() {
        if(forward){
            intake.inTake();
            intake.setMode(Intake.Modes.FORWARD);
        }
        else {
            intake.outTake();
            intake.setMode(Intake.Modes.REVERSE);
        }
    }

    public RunIntakeCommand(boolean forward, Intake intake){
        this.forward = forward;
        this.intake = intake;
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
    }
}
