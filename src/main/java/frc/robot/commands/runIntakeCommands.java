package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Modes;

public class runIntakeCommands extends Command {
    final boolean in;
    final Intake intake;

    public runIntakeCommands (boolean in, Intake intake) {
        this.in = in;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        if (in) {
            intake.intake();
            intake.setMode(Intake.Modes.INTAKE);
        }
        else {
            intake.outtake();
            intake.setMode(Intake.Modes.OUTTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
        intake.setMode(Intake.Modes.STOPPED);
    }
}
