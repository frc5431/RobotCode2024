package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class runIntakeCommands extends Command {
    final boolean in;
    final Intake intake;

    public runIntakeCommands (boolean in, Intake intake) {
        this.in = in;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        if (in) intake.intake();
        else intake.outtake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
         intake.run(0);
    }
}
